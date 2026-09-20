import * as THREE from 'three';
import { rigidBody } from 'crashcat';

const _tmpVec = new THREE.Vector3();
const _forward = new THREE.Vector3();
const _right = new THREE.Vector3();
const _zAxis = new THREE.Vector3();
const _newZ = new THREE.Vector3();
const _mat4 = new THREE.Matrix4();
const _quat = new THREE.Quaternion();
const _up = new THREE.Vector3( 0, 1, 0 );

const SPEED_SCALE = 12.5;
export const MAX_SPEED = 1.15;
export const REVERSE_MAX = -0.55;

// Vehicle physical constants
const VEHICLE_MASS = 750; // kg
const GRAVITY = 9.81; // m/s^2
const WHEELBASE = 0.65; // m (front-to-rear axle distance)
const DIST_A = 0.32; // m (CG to front axle)
const DIST_B = 0.33; // m (CG to rear axle)
const CG_HEIGHT = 0.22; // m (height of mass center)
const TOTAL_WEIGHT = VEHICLE_MASS * GRAVITY; // N

// Procedural multi-gear transmission configuration
const GEAR_RATIOS = [
	{ gear: 1, maxSpeedRatio: 0.28, torqueMult: 1.42, downshiftAt: 0.00 },
	{ gear: 2, maxSpeedRatio: 0.58, torqueMult: 1.16, downshiftAt: 0.22 },
	{ gear: 3, maxSpeedRatio: 0.84, torqueMult: 0.94, downshiftAt: 0.50 },
	{ gear: 4, maxSpeedRatio: 1.00, torqueMult: 0.74, downshiftAt: 0.76 }
];

// Pacejka 'Magic Formula' non-linear tire lateral slip characteristic
function pacejkaLateral( alpha, B = 9.0, C = 1.35, D = 1.0, E = - 0.15 ) {

	const Ba = B * alpha;
	return D * Math.sin( C * Math.atan( Ba - E * ( Ba - Math.atan( Ba ) ) ) );

}

function lerpAngle( a, b, t ) {

	let diff = b - a;
	while ( diff > Math.PI ) diff -= Math.PI * 2;
	while ( diff < -Math.PI ) diff += Math.PI * 2;
	return a + diff * t;

}

export class Vehicle {

	constructor() {

		this.linearSpeed = 0;
		this.angularSpeed = 0;
		this.acceleration = 0;
		this.steerAngle = 0;

		this.spherePos = new THREE.Vector3( 3.5, 0.5, 5 );
		this.sphereVel = new THREE.Vector3();

		this.rigidBody = null;
		this.physicsWorld = null;

		this.modelVelocity = new THREE.Vector3();
		this.prevModelPos = new THREE.Vector3( 3.5, 0, 5 );
		this.initializedModelPos = false;

		this.container = new THREE.Group();
		this.bodyNode = null;
		this.wheels = [];
		this.wheelFL = null;
		this.wheelFR = null;
		this.wheelBL = null;
		this.wheelBR = null;

		this.inputX = 0;
		this.inputZ = 0;

		this.driftIntensity = 0;
		this.slipAngle = 0;

		// Internal physical velocity & acceleration state
		this.vx = 0; // Longitudinal velocity (m/s)
		this.vy = 0; // Lateral velocity (m/s)
		this.yawRate = 0; // Yaw angular rate (rad/s)
		this.longAccel = 0; // Longitudinal acceleration (m/s^2)
		this.latAccel = 0; // Lateral acceleration (m/s^2)
		this.effectiveSteer = 0; // Steering rack angle (rad)
		this.wheelSpinSlip = 0; // Rear wheelspin slip ratio
		this.brakeLockSlip = 0; // Brake lockup slip ratio
		this.lastAppliedVel = new THREE.Vector3(); // For contact impulse detection

		// Procedural powertrain & transmission state
		this.currentGear = 1;
		this.engineRpm = 0.15;
		this.turboSpool = 0.0;
		this.effectiveThrottle = 0.0;
		this.shiftCooldown = 0.0;
		this.shiftCutTimer = 0.0;
		this.proceduralSquat = 0.0;
		this.engineVibration = 0.0;

		// Suspension spring state (2nd-order damped harmonic oscillators)
		this.bodyPitch = 0;
		this.pitchVel = 0;
		this.bodyRoll = 0;
		this.rollVel = 0;
		this.bodyHeave = 0.3;
		this.heaveVel = 0;

	}

	init( model ) {

		const vehicleModel = model.clone();

		this.container.add( vehicleModel );

		// Find body and wheel nodes
		vehicleModel.traverse( ( child ) => {

			const name = child.name.toLowerCase();

			if ( name === 'body' ) {

				child.rotation.order = 'YXZ';
				this.bodyNode = child;

			} else if ( name.includes( 'wheel' ) ) {

				child.rotation.order = 'YXZ';
				this.wheels.push( child );

				if ( name.includes( 'front' ) && name.includes( 'left' ) ) this.wheelFL = child;
				if ( name.includes( 'front' ) && name.includes( 'right' ) ) this.wheelFR = child;
				if ( name.includes( 'back' ) && name.includes( 'left' ) ) this.wheelBL = child;
				if ( name.includes( 'back' ) && name.includes( 'right' ) ) this.wheelBR = child;

			}

			if ( child.isMesh ) {

				child.castShadow = true;
				child.receiveShadow = true;

			}

		} );

		return this.container;

	}

	update( dt, controlsInput ) {

		this.inputX = controlsInput.x;
		this.inputZ = controlsInput.z;

		const prevSpeed = this.linearSpeed;

		if ( controlsInput.touchActive && ( this.inputX !== 0 || this.inputZ !== 0 ) ) {

			// Touch control: direct directional heading with automatic race throttle
			const targetAngle = Math.atan2( this.inputX, this.inputZ );
			_quat.setFromAxisAngle( _up, targetAngle );
			this.container.quaternion.slerp( _quat, 1 - Math.exp( - 4 * dt ) );

			_forward.set( 0, 0, 1 ).applyQuaternion( this.container.quaternion );
			const cross = _forward.x * this.inputZ - _forward.z * this.inputX;
			this.inputX = THREE.MathUtils.clamp( - cross * 2.2, - 1, 1 );

		}

		_forward.set( 0, 0, 1 ).applyQuaternion( this.container.quaternion );
		_forward.y = 0;
		_forward.normalize();

		_right.set( 1, 0, 0 ).applyQuaternion( this.container.quaternion );
		_right.y = 0;
		_right.normalize();

		// Read Crashcat physics state and detect barrier collision impulses
		if ( this.rigidBody ) {

			const pos = this.rigidBody.position;
			this.spherePos.set( pos[ 0 ], pos[ 1 ], pos[ 2 ] );

			const physVel = this.rigidBody.motionProperties.linearVelocity;
			this.sphereVel.set( physVel[ 0 ], physVel[ 1 ], physVel[ 2 ] );

			const diffVx = this.sphereVel.x - this.lastAppliedVel.x;
			const diffVz = this.sphereVel.z - this.lastAppliedVel.z;
			const fwdImpulse = diffVx * _forward.x + diffVz * _forward.z;
			const rightImpulse = diffVx * _right.x + diffVz * _right.z;

			// Wall collision: absorb forward momentum, impart restitution bounce and glancing deflection
			if ( fwdImpulse < - 0.8 ) {

				this.vx = Math.max( - 0.4, this.vx + fwdImpulse * 0.75 );

			}

			if ( Math.abs( rightImpulse ) > 0.4 ) {

				this.vy += rightImpulse * 0.55;
				this.yawRate += ( - rightImpulse * 0.55 );

			}

		}

		// 1. Dynamic longitudinal weight transfer: normal load shifts between front and rear axles
		const dFz = ( VEHICLE_MASS * this.longAccel * CG_HEIGHT ) / WHEELBASE;
		const fzf = THREE.MathUtils.clamp( TOTAL_WEIGHT * ( DIST_B / WHEELBASE ) - dFz, 800, TOTAL_WEIGHT * 0.85 );
		const fzr = THREE.MathUtils.clamp( TOTAL_WEIGHT * ( DIST_A / WHEELBASE ) + dFz, 800, TOTAL_WEIGHT * 0.85 );

		// 2. Procedural powertrain and braking forces
		let fxf = 0;
		let fxr = 0;
		this.wheelSpinSlip = 0;
		this.brakeLockSlip = 0;

		const MAX_DRIVE_FORCE = 3800; // N calibrated base torque
		const MAX_BRAKE_FORCE = 8200; // N
		const MU_X = 1.45;

		const absSpeed = Math.abs( this.vx );
		const speedRatio = THREE.MathUtils.clamp( absSpeed / ( MAX_SPEED * SPEED_SCALE ), 0, 1 );

		// Update shift timers
		this.shiftCooldown = Math.max( 0, this.shiftCooldown - dt );
		this.shiftCutTimer = Math.max( 0, this.shiftCutTimer - dt );

		if ( this.inputZ > 0.05 ) {

			// A. Procedural smooth throttle travel response
			this.effectiveThrottle = THREE.MathUtils.lerp( this.effectiveThrottle, this.inputZ, dt * 14.0 );

			// B. Procedural turbo boost pressure spooling
			const turboTarget = this.effectiveThrottle > 0.15 ? Math.min( 1.0, this.effectiveThrottle * 1.15 ) : 0.0;
			this.turboSpool = THREE.MathUtils.lerp( this.turboSpool, turboTarget, dt * ( this.turboSpool < turboTarget ? 3.8 : 7.5 ) );

			// C. Procedural multi-gear transmission progression & auto-shifting
			let gearIdx = this.currentGear - 1;
			if ( this.shiftCooldown === 0 ) {

				// Check upshift
				if ( gearIdx < GEAR_RATIOS.length - 1 && speedRatio > GEAR_RATIOS[ gearIdx ].maxSpeedRatio * 0.88 && this.engineRpm > 0.82 ) {

					this.currentGear ++;
					gearIdx ++;
					this.shiftCooldown = 0.38;
					this.shiftCutTimer = 0.08;
					this.engineRpm = 0.48;

				} else if ( gearIdx > 0 && speedRatio < GEAR_RATIOS[ gearIdx ].downshiftAt ) {

					this.currentGear --;
					gearIdx --;
					this.shiftCooldown = 0.25;
					this.engineRpm = 0.75;

				}

			}

			// D. Procedural engine RPM modeling
			const currentGearDef = GEAR_RATIOS[ gearIdx ];
			const prevGearMax = gearIdx > 0 ? GEAR_RATIOS[ gearIdx - 1 ].maxSpeedRatio : 0.0;
			const inGearRatio = THREE.MathUtils.clamp( ( speedRatio - prevGearMax ) / Math.max( 0.04, currentGearDef.maxSpeedRatio - prevGearMax ), 0, 1 );
			const targetRpm = inGearRatio * 0.76 + this.effectiveThrottle * 0.24;
			this.engineRpm = THREE.MathUtils.lerp( this.engineRpm, targetRpm, dt * 9.0 );

			// E. Non-linear procedural powerband torque curve
			// Sinusoidal torque curve with peak mid-range pull and sweet-spot delivery
			const powerband = 0.74 + 0.46 * Math.sin( Math.min( 1.0, this.engineRpm * 1.15 ) * Math.PI * 0.72 );
			const boostTorqueMult = 1.0 + this.turboSpool * 0.32;
			const shiftTorqueMult = this.shiftCutTimer > 0 ? 0.20 : 1.0;

			// Top-speed progressive governor taper
			const topEndTaper = Math.max( 0.0, 1.0 - Math.pow( speedRatio, 4.0 ) );

			const driveTorque = MAX_DRIVE_FORCE * currentGearDef.torqueMult * powerband * boostTorqueMult * shiftTorqueMult * topEndTaper * this.effectiveThrottle;
			const maxRearTraction = MU_X * fzr;

			if ( driveTorque > maxRearTraction ) {

				// Launch tire burnout / wheelspin
				fxr = maxRearTraction;
				this.wheelSpinSlip = THREE.MathUtils.clamp( ( driveTorque - maxRearTraction ) / maxRearTraction, 0.0, 1.5 );

			} else {

				fxr = driveTorque;

			}

		} else if ( this.inputZ < - 0.05 ) {

			this.effectiveThrottle = THREE.MathUtils.lerp( this.effectiveThrottle, 0, dt * 18.0 );
			this.turboSpool = THREE.MathUtils.lerp( this.turboSpool, 0, dt * 8.0 );

			// Downshift gear to 1 in reverse / deceleration
			if ( speedRatio < 0.30 ) this.currentGear = 1;

			if ( this.vx > 0.25 ) {

				// Active hydraulic braking with 65% front / 35% rear balance
				const demandedBrake = Math.abs( this.inputZ ) * MAX_BRAKE_FORCE;
				fxf = - 0.65 * demandedBrake;
				fxr = - 0.35 * demandedBrake;

				const maxFrontBrake = MU_X * fzf;
				if ( Math.abs( fxf ) > maxFrontBrake ) {

					fxf = - maxFrontBrake;
					this.brakeLockSlip = 0.85;

				}

			} else {

				// Smooth powerful reverse gear with controlled speed ceiling
				const reverseRatio = THREE.MathUtils.clamp( Math.abs( this.vx ) / ( Math.abs( REVERSE_MAX ) * SPEED_SCALE ), 0, 1 );
				const reverseTaper = Math.max( 0.0, 1.0 - Math.pow( reverseRatio, 3.0 ) );
				fxr = this.inputZ * 3200 * reverseTaper;

			}

		} else {

			this.effectiveThrottle = THREE.MathUtils.lerp( this.effectiveThrottle, 0, dt * 15.0 );
			this.turboSpool = THREE.MathUtils.lerp( this.turboSpool, 0, dt * 6.0 );

			if ( speedRatio < 0.25 ) this.currentGear = 1;

			// Rolling resistance and engine compression braking
			const rollingResist = 0.025 * TOTAL_WEIGHT * Math.sign( this.vx );
			const engineBraking = 160 * Math.sign( this.vx );
			fxr = - ( rollingResist + engineBraking );
			if ( Math.abs( this.vx ) < 0.05 ) this.vx = 0;

		}

		// Aerodynamic drag force: F_drag = 0.5 * rho * Cd * A * v^2
		const aeroDrag = 0.5 * 1.225 * 0.38 * 0.90 * this.vx * Math.abs( this.vx );
		const totalFx = fxf + fxr - aeroDrag;
		this.longAccel = totalFx / VEHICLE_MASS;
		this.vx += this.longAccel * dt;

		// 3. Speed-sensitive steering rack dynamics
		const currentAbsSpeed = Math.abs( this.vx );
		const currentSpeedRatio = THREE.MathUtils.clamp( currentAbsSpeed / ( MAX_SPEED * SPEED_SCALE ), 0, 1 );
		const steerSensitivity = THREE.MathUtils.lerp( 5.2, 3.4, currentSpeedRatio );

		// In reverse or shifting into reverse, reverse the yaw rotation direction
		let direction = 1;
		if ( this.vx < - 0.05 || ( this.inputZ < - 0.05 && this.vx <= 0.05 ) ) {

			direction = - 1;

		} else if ( this.vx > 0.05 ) {

			direction = 1;

		}

		// Provide full steering crawl authority at standstill/low-speeds so stuck cars can turn their wheels out
		const isActivelySteeringOrDriving = ( Math.abs( this.inputX ) > 0.05 || Math.abs( this.inputZ ) > 0.05 );
		const minSteerGrip = isActivelySteeringOrDriving ? 0.75 : 0.0;
		const speedGrip = THREE.MathUtils.clamp( Math.max( currentAbsSpeed / 0.18, minSteerGrip ), 0.0, 1.0 );

		// Target yaw angular rate
		const targetYaw = - this.inputX * steerSensitivity * direction * speedGrip;
		const steerSmoothing = Math.sign( this.yawRate ) !== Math.sign( targetYaw ) && this.inputX !== 0 ? 16 : 10;
		this.yawRate = THREE.MathUtils.lerp( this.yawRate, targetYaw, dt * ( speedGrip > 0.05 ? steerSmoothing : 16 ) );

		// Front wheel rack steering angle (tapers slightly at high speed for high-speed stability)
		const targetSteerAngle = - this.inputX * THREE.MathUtils.lerp( 0.55, 0.32, currentSpeedRatio );
		this.effectiveSteer = THREE.MathUtils.lerp( this.effectiveSteer, targetSteerAngle, dt * 14 );
		this.steerAngle = this.effectiveSteer;

		// 4. Rear slip angle and Pacejka lateral tire forces
		const vr = this.vy - DIST_B * this.yawRate;
		const speedDenom = Math.max( 0.4, currentAbsSpeed );
		const alphaR = Math.atan2( vr, speedDenom );
		this.slipAngle = Math.abs( alphaR );

		// Authentic drift damping: tight grip at low slip angles, progressive sliding during drifts
		const driftSlip = Math.min( 1.0, this.slipAngle / 0.35 );
		const lateralGripCoeff = THREE.MathUtils.lerp( 28.0, 9.0, driftSlip );
		this.vy *= Math.exp( - lateralGripCoeff * dt );

		// 5. Container orientation and world-space integration
		this.angularSpeed = this.yawRate;
		this.container.rotateY( this.angularSpeed * dt );

		// Keep container upright along world Y axis
		_tmpVec.set( 0, 1, 0 ).applyQuaternion( this.container.quaternion );
		if ( _tmpVec.y > 0.5 ) {

			const targetQuat = this.alignWithY( this.container.quaternion, _up );
			this.container.quaternion.slerp( targetQuat, 0.25 );

		}

		_forward.set( 0, 0, 1 ).applyQuaternion( this.container.quaternion );
		_right.set( 1, 0, 0 ).applyQuaternion( this.container.quaternion );

		const newVx = _forward.x * this.vx + _right.x * this.vy;
		const newVz = _forward.z * this.vx + _right.z * this.vy;
		this.lastAppliedVel.set( newVx, this.sphereVel.y, newVz );

		if ( this.rigidBody ) {

			rigidBody.setLinearVelocity( this.physicsWorld, this.rigidBody, [ newVx, this.sphereVel.y, newVz ] );

			// Synchronize sphere rolling angular velocity
			const rollSpeed = this.vx / 0.5;
			rigidBody.setAngularVelocity( this.physicsWorld, this.rigidBody, [
				_right.x * rollSpeed,
				0,
				_right.z * rollSpeed
			] );

		}

		this.linearSpeed = this.vx / SPEED_SCALE;

		// Calculate smoothed forward acceleration for audio and visual cues
		const forwardAccel = ( this.linearSpeed - prevSpeed ) / Math.max( dt, 0.001 );
		this.acceleration = THREE.MathUtils.lerp(
			this.acceleration,
			this.linearSpeed + ( 0.2 * this.linearSpeed * Math.abs( this.linearSpeed ) ),
			dt * 8
		);

		// Void recovery
		if ( this.spherePos.y < - 10 ) {

			this.repositionTo( 3.5, 0.5, 5, 0 );

		}

		this.container.position.set(
			this.spherePos.x,
			this.spherePos.y - 0.5,
			this.spherePos.z
		);

		if ( ! this.initializedModelPos ) {

			this.initializedModelPos = true;
			this.prevModelPos.copy( this.container.position );
			this.modelVelocity.set( 0, 0, 0 );

		} else if ( dt > 0 ) {

			this.modelVelocity.subVectors( this.container.position, this.prevModelPos ).divideScalar( dt );
			this.prevModelPos.copy( this.container.position );

		}

		// Combined drift intensity calculated from real tire slip, wheelspin, and brake skid
		this.driftIntensity = THREE.MathUtils.clamp( this.slipAngle * 2.8 + this.wheelSpinSlip * 0.8 + this.brakeLockSlip, 0.0, 2.0 );

		this.updateBodySuspension( dt, forwardAccel );
		this.updateWheels( dt );

	}

	repositionTo( x, y, z, yawAngle = 0 ) {

		if ( this.rigidBody && this.physicsWorld ) {

			rigidBody.setPosition( this.physicsWorld, this.rigidBody, [ x, y, z ], false );
			rigidBody.setLinearVelocity( this.physicsWorld, this.rigidBody, [ 0, 0, 0 ] );
			rigidBody.setAngularVelocity( this.physicsWorld, this.rigidBody, [ 0, 0, 0 ] );

		}

		this.spherePos.set( x, y, z );
		this.sphereVel.set( 0, 0, 0 );
		this.prevModelPos.set( x, 0, z );
		this.modelVelocity.set( 0, 0, 0 );
		this.lastAppliedVel.set( 0, 0, 0 );
		this.vx = 0;
		this.vy = 0;
		this.yawRate = 0;
		this.linearSpeed = 0;
		this.angularSpeed = 0;
		this.acceleration = 0;
		this.bodyPitch = 0;
		this.pitchVel = 0;
		this.bodyRoll = 0;
		this.rollVel = 0;
		this.effectiveSteer = 0;
		this.steerAngle = 0;
		this.container.rotation.set( 0, yawAngle, 0 );
		this.container.quaternion.setFromAxisAngle( _up, yawAngle );
		this.container.position.set( x, y - 0.5, z );

	}

	alignWithY( quaternion, newY ) {

		_zAxis.set( 0, 0, 1 ).applyQuaternion( quaternion );
		const xAxis = _tmpVec.crossVectors( _zAxis, newY ).negate().normalize();
		_newZ.crossVectors( xAxis, newY ).normalize();

		_mat4.makeBasis( xAxis, newY, _newZ );
		return _quat.setFromRotationMatrix( _mat4 );

	}

	updateBodySuspension( dt, forwardAccel ) {

		if ( ! this.bodyNode ) return;

		// 1. Procedural longitudinal pitch (squat under torque & launch, shift-jolt, dive under braking)
		const shiftJolt = this.shiftCutTimer > 0 ? 0.035 : 0.0;
		const targetPitch = THREE.MathUtils.clamp( - forwardAccel * 0.048 - shiftJolt, - 0.16, 0.14 );
		const pitchForce = ( targetPitch - this.bodyPitch ) * 38;
		this.pitchVel = ( this.pitchVel + pitchForce * dt ) * Math.max( 0, 1 - 8 * dt );
		this.bodyPitch += this.pitchVel * dt;

		// 2. Centrifugal body roll (chassis leans outward in corners with spring-damper rebound)
		const targetRoll = THREE.MathUtils.clamp( - ( this.angularSpeed * 0.06 ) * ( 0.6 + Math.abs( this.linearSpeed ) * 0.4 ), - 0.18, 0.18 );
		const rollForce = ( targetRoll - this.bodyRoll ) * 30;
		this.rollVel = ( this.rollVel + rollForce * dt ) * Math.max( 0, 1 - 7 * dt );
		this.bodyRoll += this.rollVel * dt;

		// 3. Dynamic vertical heave + procedural engine idle/rev micro-vibrations
		const engineRev = Math.max( 0.15, this.engineRpm );
		const vibration = Math.sin( performance.now() * 0.05 * ( 0.8 + engineRev * 2.0 ) ) * ( 0.0015 * engineRev );
		const targetHeave = 0.3 + ( Math.abs( this.pitchVel ) + Math.abs( this.rollVel ) ) * 0.02 + vibration;
		this.heaveVel = ( this.heaveVel + ( targetHeave - this.bodyHeave ) * 25 * dt ) * Math.max( 0, 1 - 9 * dt );
		this.bodyHeave += this.heaveVel * dt;

		this.bodyNode.rotation.x = this.bodyPitch;
		this.bodyNode.rotation.z = this.bodyRoll;
		this.bodyNode.position.y = this.bodyHeave;

	}

	updateWheels( dt ) {

		// Wheels roll proportional to physical linear speed + burnout spin
		const wheelRoll = ( this.linearSpeed * 22 + this.wheelSpinSlip * 18 ) * dt;

		for ( const wheel of this.wheels ) {

			wheel.rotation.x += wheelRoll;

		}

		// Ackermann steering geometry + dynamic cornering camber on front wheels
		if ( this.wheelFL ) {

			this.wheelFL.rotation.y = this.steerAngle * ( this.steerAngle > 0 ? 1.08 : 0.94 );
			this.wheelFL.rotation.z = - this.steerAngle * 0.08; // Dynamic camber

		}

		if ( this.wheelFR ) {

			this.wheelFR.rotation.y = this.steerAngle * ( this.steerAngle < 0 ? 1.08 : 0.94 );
			this.wheelFR.rotation.z = - this.steerAngle * 0.08; // Dynamic camber

		}

	}

}

