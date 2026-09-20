import * as THREE from 'three';
import { CELL_RAW, GRID_SCALE, TRACK_CELLS, ORIENT_DEG, computeSpawnPosition } from './Track.js';

const S = CELL_RAW * GRID_SCALE; // Cell size in world units (~7.4925)
const HALF_S = S / 2;

// Port directions: 0: North (-Z), 1: East (+X), 2: South (+Z), 3: West (-X)
const DX = [ 0, 1, 0, -1 ];
const DZ = [ -1, 0, 1, 0 ];

/**
 * Returns [portA, portB] connections for a piece.
 */
function getPiecePorts( type, godotOrient ) {

	const deg = ORIENT_DEG[ godotOrient ] ?? 0;

	if ( type === 'track-straight' || type === 'track-bump' || type === 'track-finish' ) {

		if ( deg === 0 || deg === 180 ) {

			return [ 0, 2 ];

		} else {

			return [ 1, 3 ];

		}

	} else if ( type === 'track-corner' ) {

		if ( deg === 0 ) return [ 3, 2 ];
		if ( deg === 90 ) return [ 2, 1 ];
		if ( deg === 180 ) return [ 1, 0 ];
		if ( deg === 270 ) return [ 0, 3 ];

	}

	return [ 0, 2 ];

}

/**
 * Generates smooth, mathematically exact centerline waypoints through a track cell.
 * Correctly accounts for Three.js coordinates (+X East, +Z South).
 */
function getCellWaypoints( cell, entryPort, exitPort ) {

	const [ gx, gz, type, godotOrient ] = cell;
	const cx = ( gx + 0.5 ) * S;
	const cz = ( gz + 0.5 ) * S;

	const points = [];

	if ( type === 'track-corner' ) {

		const deg = ORIENT_DEG[ godotOrient ] ?? 0;
		let pivotX = cx, pivotZ = cz;
		let startAngle = 0, endAngle = 0;

		if ( deg === 0 ) {

			// Pivot at (cx - HALF_S, cz + HALF_S)
			// Port 2 (South) is angle 0
			// Port 3 (West) is angle -PI * 0.5
			pivotX = cx - HALF_S;
			pivotZ = cz + HALF_S;
			if ( entryPort === 2 ) {

				startAngle = 0;
				endAngle = - Math.PI * 0.5;

			} else {

				startAngle = - Math.PI * 0.5;
				endAngle = 0;

			}

		} else if ( deg === 90 ) {

			// Pivot at (cx + HALF_S, cz + HALF_S)
			// Port 2 (South) is angle PI
			// Port 1 (East) is angle 1.5 * PI
			pivotX = cx + HALF_S;
			pivotZ = cz + HALF_S;
			if ( entryPort === 2 ) {

				startAngle = Math.PI;
				endAngle = Math.PI * 1.5;

			} else {

				startAngle = Math.PI * 1.5;
				endAngle = Math.PI;

			}

		} else if ( deg === 180 ) {

			// Pivot at (cx + HALF_S, cz - HALF_S)
			// Port 1 (East) is angle 0.5 * PI
			// Port 0 (North) is angle PI
			pivotX = cx + HALF_S;
			pivotZ = cz - HALF_S;
			if ( entryPort === 1 ) {

				startAngle = Math.PI * 0.5;
				endAngle = Math.PI;

			} else {

				startAngle = Math.PI;
				endAngle = Math.PI * 0.5;

			}

		} else if ( deg === 270 ) {

			// Pivot at (cx - HALF_S, cz - HALF_S)
			// Port 0 (North) is angle 0
			// Port 3 (West) is angle 0.5 * PI
			pivotX = cx - HALF_S;
			pivotZ = cz - HALF_S;
			if ( entryPort === 0 ) {

				startAngle = 0;
				endAngle = Math.PI * 0.5;

			} else {

				startAngle = Math.PI * 0.5;
				endAngle = 0;

			}

		}

		const steps = 8;
		for ( let i = 0; i <= steps; i ++ ) {

			const t = i / steps;
			const angle = startAngle + ( endAngle - startAngle ) * t;
			points.push( new THREE.Vector3(
				pivotX + Math.cos( angle ) * HALF_S,
				0.0,
				pivotZ + Math.sin( angle ) * HALF_S
			) );

		}

	} else {

		const inX = cx + DX[ entryPort ] * HALF_S;
		const inZ = cz + DZ[ entryPort ] * HALF_S;
		const outX = cx + DX[ exitPort ] * HALF_S;
		const outZ = cz + DZ[ exitPort ] * HALF_S;

		points.push( new THREE.Vector3( inX, 0, inZ ) );
		points.push( new THREE.Vector3( ( inX * 2 + outX ) / 3, 0, ( inZ * 2 + outZ ) / 3 ) );
		points.push( new THREE.Vector3( ( inX + outX * 2 ) / 3, 0, ( inZ + outZ * 2 ) / 3 ) );
		points.push( new THREE.Vector3( outX, 0, outZ ) );

	}

	return points;

}

/**
 * Builds an ordered closed spline for the entire track circuit.
 */
export function buildTrackSpline( customCells ) {

	const cells = customCells || TRACK_CELLS;
	if ( ! cells || cells.length === 0 ) return null;

	const cellMap = new Map();
	let startCell = cells[ 0 ];

	for ( const c of cells ) {

		cellMap.set( `${ c[ 0 ] },${ c[ 1 ] }`, c );
		if ( c[ 2 ] === 'track-finish' ) {

			startCell = c;

		}

	}

	const spawn = computeSpawnPosition( cells );
	const startAngle = spawn.angle;
	const fwdX = Math.sin( startAngle );
	const fwdZ = Math.cos( startAngle );

	const startPorts = getPiecePorts( startCell[ 2 ], startCell[ 3 ] );
	let currentEntry = startPorts[ 0 ];
	let currentExit = startPorts[ 1 ];

	const exitFwdDot = DX[ currentExit ] * fwdX + DZ[ currentExit ] * fwdZ;
	const entryFwdDot = DX[ currentEntry ] * fwdX + DZ[ currentEntry ] * fwdZ;
	if ( entryFwdDot > exitFwdDot ) {

		currentExit = startPorts[ 0 ];
		currentEntry = startPorts[ 1 ];

	}

	const rawPoints = [];
	let currCell = startCell;
	const visited = new Set();

	for ( let step = 0; step < cells.length * 2; step ++ ) {

		const key = `${ currCell[ 0 ] },${ currCell[ 1 ] }`;
		if ( visited.has( key ) && currCell === startCell && step > 1 ) break;
		visited.add( key );

		const pts = getCellWaypoints( currCell, currentEntry, currentExit );
		for ( let i = 0; i < pts.length; i ++ ) {

			if ( rawPoints.length === 0 || rawPoints[ rawPoints.length - 1 ].distanceTo( pts[ i ] ) > 0.05 ) {

				rawPoints.push( pts[ i ] );

			}

		}

		const nextGx = currCell[ 0 ] + DX[ currentExit ];
		const nextGz = currCell[ 1 ] + DZ[ currentExit ];
		const nextKey = `${ nextGx },${ nextGz }`;
		const nextCell = cellMap.get( nextKey );

		if ( ! nextCell ) break;

		const nextEntry = ( currentExit + 2 ) % 4;
		const nextPorts = getPiecePorts( nextCell[ 2 ], nextCell[ 3 ] );
		const nextExit = ( nextPorts[ 0 ] === nextEntry ) ? nextPorts[ 1 ] : nextPorts[ 0 ];

		currCell = nextCell;
		currentEntry = nextEntry;
		currentExit = nextExit;

	}

	if ( rawPoints.length < 4 ) return null;

	if ( rawPoints[ rawPoints.length - 1 ].distanceTo( rawPoints[ 0 ] ) < 0.5 ) {

		rawPoints.pop();

	}

	return new THREE.CatmullRomCurve3( rawPoints, true, 'centripetal', 0.25 );

}

// Pre-allocated vectors for GC-free calculations
const _carPos = new THREE.Vector3();
const _forward = new THREE.Vector3();
const _right = new THREE.Vector3();
const _toTarget = new THREE.Vector3();
const _rayOrigin = new THREE.Vector3();
const _rayDir = new THREE.Vector3();
const _rayEnd = new THREE.Vector3();
const _upAxis = new THREE.Vector3( 0, 1, 0 );

// 9-Beam Precision LiDAR Configuration (Negative = Left, Positive = Right)
const LIDAR_RAYS = [
	{ id: 'fwdFar', angle: 0, range: 28.0, name: 'Center Radar' },
	{ id: 'fwdLeft', angle: - 16, range: 20.0, name: 'Forward Left' },
	{ id: 'fwdRight', angle: 16, range: 20.0, name: 'Forward Right' },
	{ id: 'diagLeft', angle: - 38, range: 12.0, name: 'Diagonal Left' },
	{ id: 'diagRight', angle: 38, range: 12.0, name: 'Diagonal Right' },
	{ id: 'sideLeft', angle: - 85, range: 6.5, name: 'Side Left' },
	{ id: 'sideRight', angle: 85, range: 6.5, name: 'Side Right' },
	{ id: 'rearLeft', angle: - 145, range: 5.0, name: 'Rear Left' },
	{ id: 'rearRight', angle: 145, range: 5.0, name: 'Rear Right' },
];

export const AI_PRESETS = {
	racing: {
		name: 'Apex Hunter (Racing)',
		steerGain: 3.2,
		lookaheadBase: 3.6,
		cornerBraking: 0.85,
		wallAvoidance: 1.4,
		maxThrottle: 1.0,
		targetSpeed: 1.15,
		wallSafetyMargin: 1.8,
		description: 'Fast, smooth racing line with smart cornering and gentle wall cushioning.',
	},
	safe: {
		name: 'Cruise Control (Safe)',
		steerGain: 2.8,
		lookaheadBase: 4.2,
		cornerBraking: 1.2,
		wallAvoidance: 2.0,
		maxThrottle: 0.85,
		targetSpeed: 0.85,
		wallSafetyMargin: 2.2,
		description: 'Strict track center adherence, steady speed, zero drift.',
	},
	drift: {
		name: 'Tokyo Drift (Sliding)',
		steerGain: 4.2,
		lookaheadBase: 2.8,
		cornerBraking: 0.4,
		wallAvoidance: 1.0,
		maxThrottle: 1.05,
		targetSpeed: 1.15,
		wallSafetyMargin: 1.5,
		description: 'High entry speed with late turn-in.',
	},
	speed: {
		name: 'Speed Demon (Max V)',
		steerGain: 3.5,
		lookaheadBase: 3.4,
		cornerBraking: 0.5,
		wallAvoidance: 1.2,
		maxThrottle: 1.0,
		targetSpeed: 1.18,
		wallSafetyMargin: 1.6,
		description: 'Aggressive acceleration with minimum braking on straights.',
	}
};

export class AIDriver {

	constructor( scene, customCells, wallMeshes, obstacleMeshes, camera ) {

		this.scene = scene;
		this.camera = camera || null;
		this.wallMeshes = wallMeshes || [];
		this.obstacleMeshes = obstacleMeshes || [];
		this.allObstacles = [ ...this.wallMeshes, ...this.obstacleMeshes ];
		this.enabled = false;
		this.spline = buildTrackSpline( customCells );

		// Interactive Mouse Cursor Hazard Test
		this.mouseObstacleActive = false;
		this.mouseObstaclePos = new THREE.Vector3( 0, 0, 0 );
		this.mouseObstacleRadius = 1.0;
		this.mouseObstacleHitMesh = null;
		this.mouseObstacleGroup = null;
		this.mouseObstacleTopper = null;
		this.mouseObstacleRing = null;
		this.mouseObstacleOuterRing = null;
		this.lastVehicle = null;
		this.toastBanner = null;
		this.toastTimeout = null;
		this.mouseHazardBtn = null;

		// Sample the spline evenly along arc length
		this.samplesCount = 500;
		this.samples = [];
		this.totalSplineLength = 100;

		if ( this.spline ) {

			this.totalSplineLength = this.spline.getLength();
			for ( let i = 0; i < this.samplesCount; i ++ ) {

				const u = i / this.samplesCount;
				const pt = this.spline.getPointAt( u );
				const tangent = this.spline.getTangentAt( u ).normalize();
				this.samples.push( {
					u,
					dist: u * this.totalSplineLength,
					point: pt,
					tangent: tangent
				} );

			}

		}

		this.closestSampleIdx = 0;

		// Multi-Phase 3-Point Unstuck State Machine
		this.recoveryPhase = 0; // 0: Normal, 1: Reverse, 2: Pivot Forward
		this.recoveryTimer = 0;
		this.recoverySteer = 0;
		this.stuckWatchTimer = 0;
		this.recoveryCount = 0;
		this.totalStuckDuration = 0;
		this.decisionState = 'RACING_LINE';
		this.lastPosition = new THREE.Vector3();
		this.smoothedSteer = 0;

		// Live editable configuration
		this.config = {
			...AI_PRESETS.racing,
			showSensors: true,
			showTuner: false,
			activePreset: 'racing',
			customCode: '',
			useCustomCode: false,
		};

		// Realtime live sensor readouts
		this.sensors = {
			fwdFar: 25,
			fwdLeft: 16,
			fwdRight: 16,
			diagLeft: 9,
			diagRight: 9,
			sideLeft: 5.5,
			sideRight: 5.5,
			rearLeft: 4.5,
			rearRight: 4.5,
			trackOffset: 0,
			curvature: 0,
			nearestWallDist: 25,
		};

		this.telemetry = {
			trueSpeed: 0,
			steer: 0,
			throttle: 0,
			status: 'STANDBY',
			lookaheadDist: 0,
			mouseObstacleDist: Infinity,
		};

		// Raycasting
		this.raycaster = new THREE.Raycaster();
		this.raycaster.far = 28;

		this.customLogicFunction = null;

		// Visual Sensors in 3D Scene
		this.buildVisualSensors();

		// Interactive Mouse Cursor Hazard Visuals & Listener
		this.buildMouseObstacleVisuals();
		this.setupMouseRaycaster();

		// HUD & Hacker Studio UI
		if ( typeof document !== 'undefined' ) {

			this.buildHUD();

		}

		// Global dev console injection
		if ( typeof window !== 'undefined' ) {

			window.autopilot = this;

		}

	}

	addObstacle( mesh ) {

		if ( ! mesh ) return;
		this.obstacleMeshes.push( mesh );
		this.allObstacles.push( mesh );

	}

	setObstacles( meshes ) {

		this.obstacleMeshes = meshes || [];
		this.allObstacles = [ ...this.wallMeshes, ...this.obstacleMeshes ];
		if ( this.mouseObstacleActive && this.mouseObstacleHitMesh ) {

			this.allObstacles.push( this.mouseObstacleHitMesh );

		}

	}

	buildMouseObstacleVisuals() {

		if ( ! this.scene ) return;

		this.mouseObstacleGroup = new THREE.Group();
		this.mouseObstacleGroup.name = 'MouseHazardObstacle';
		this.mouseObstacleGroup.visible = false;

		// 1. Collision Hit Mesh for LiDAR 9-ray perception
		const hitGeom = new THREE.CylinderGeometry( this.mouseObstacleRadius, this.mouseObstacleRadius, 2.4, 16 );
		hitGeom.translate( 0, 1.2, 0 );
		const hitMat = new THREE.MeshBasicMaterial( {
			color: 0xff3b00,
			wireframe: false,
			transparent: true,
			opacity: 0.15,
			depthWrite: false,
		} );
		this.mouseObstacleHitMesh = new THREE.Mesh( hitGeom, hitMat );
		this.mouseObstacleHitMesh.name = 'MouseObstacleHitCollider';
		this.mouseObstacleGroup.add( this.mouseObstacleHitMesh );

		// 2. High-visibility Warning Pylon Pillar
		const pylonGeom = new THREE.CylinderGeometry( 0.45, 0.65, 1.6, 16 );
		pylonGeom.translate( 0, 0.8, 0 );
		const pylonMat = new THREE.MeshStandardMaterial( {
			color: 0xff5500,
			roughness: 0.25,
			metalness: 0.1,
			emissive: 0xff2a00,
			emissiveIntensity: 0.6,
		} );
		const pylon = new THREE.Mesh( pylonGeom, pylonMat );
		pylon.castShadow = true;
		this.mouseObstacleGroup.add( pylon );

		// White reflective warning band
		const stripeGeom = new THREE.CylinderGeometry( 0.48, 0.54, 0.32, 16 );
		stripeGeom.translate( 0, 0.85, 0 );
		const stripeMat = new THREE.MeshStandardMaterial( {
			color: 0xffffff,
			roughness: 0.1,
			emissive: 0xffffff,
			emissiveIntensity: 0.4,
		} );
		const stripe = new THREE.Mesh( stripeGeom, stripeMat );
		this.mouseObstacleGroup.add( stripe );

		// 3. Rotating Holographic Warning Diamond Beacon
		const topperGeom = new THREE.OctahedronGeometry( 0.35, 0 );
		topperGeom.translate( 0, 2.0, 0 );
		const topperMat = new THREE.MeshStandardMaterial( {
			color: 0xffd000,
			emissive: 0xffaa00,
			emissiveIntensity: 1.2,
			wireframe: true,
		} );
		this.mouseObstacleTopper = new THREE.Mesh( topperGeom, topperMat );
		this.mouseObstacleGroup.add( this.mouseObstacleTopper );

		// 4. Ground Pulsing Radar Danger Rings
		const innerRingGeom = new THREE.RingGeometry( 0.85, 1.15, 32 );
		innerRingGeom.rotateX( - Math.PI / 2 );
		const innerRingMat = new THREE.MeshBasicMaterial( {
			color: 0xef4444,
			side: THREE.DoubleSide,
			transparent: true,
			opacity: 0.75,
			depthWrite: false,
		} );
		this.mouseObstacleRing = new THREE.Mesh( innerRingGeom, innerRingMat );
		this.mouseObstacleRing.position.y = 0.05;
		this.mouseObstacleGroup.add( this.mouseObstacleRing );

		const outerRingGeom = new THREE.RingGeometry( 1.45, 1.58, 32 );
		outerRingGeom.rotateX( - Math.PI / 2 );
		const outerRingMat = new THREE.MeshBasicMaterial( {
			color: 0xf59e0b,
			side: THREE.DoubleSide,
			transparent: true,
			opacity: 0.45,
			depthWrite: false,
		} );
		this.mouseObstacleOuterRing = new THREE.Mesh( outerRingGeom, outerRingMat );
		this.mouseObstacleOuterRing.position.y = 0.04;
		this.mouseObstacleGroup.add( this.mouseObstacleOuterRing );

		this.scene.add( this.mouseObstacleGroup );

	}

	setupMouseRaycaster() {

		if ( typeof window === 'undefined' ) return;

		const groundPlane = new THREE.Plane( new THREE.Vector3( 0, 1, 0 ), 0 );
		const mouseRaycaster = new THREE.Raycaster();
		const mouseCoord = new THREE.Vector2();
		const intersection = new THREE.Vector3();

		const onPointerMove = ( e ) => {

			if ( ! this.mouseObstacleActive ) return;
			if ( ! this.camera ) {

				if ( this.scene ) {

					this.scene.traverse( ( obj ) => {

						if ( obj.isPerspectiveCamera && ! this.camera ) this.camera = obj;

					} );

				}

			}

			if ( ! this.camera ) return;

			mouseCoord.x = ( e.clientX / window.innerWidth ) * 2 - 1;
			mouseCoord.y = - ( e.clientY / window.innerHeight ) * 2 + 1;

			mouseRaycaster.setFromCamera( mouseCoord, this.camera );
			if ( mouseRaycaster.ray.intersectPlane( groundPlane, intersection ) ) {

				this.setMouseObstaclePosition( intersection.x, intersection.z );

			}

		};

		window.addEventListener( 'pointermove', onPointerMove, { passive: true } );
		window.addEventListener( 'pointerdown', onPointerMove, { passive: true } );

	}

	setMouseObstaclePosition( x, z ) {

		this.mouseObstaclePos.set( x, 0, z );
		if ( this.mouseObstacleGroup ) {

			this.mouseObstacleGroup.position.set( x, 0, z );

		}

	}

	toggleMouseObstacle( forceState ) {

		this.mouseObstacleActive = ( forceState !== undefined ) ? forceState : ! this.mouseObstacleActive;

		if ( this.mouseObstacleActive ) {

			if ( this.mouseObstacleGroup ) {

				this.mouseObstacleGroup.visible = true;

			}

			if ( this.mouseObstacleHitMesh && ! this.allObstacles.includes( this.mouseObstacleHitMesh ) ) {

				this.allObstacles.push( this.mouseObstacleHitMesh );

			}

			if ( ! this.enabled ) {

				this.toggle( true );

			}

			this.showToast( '🎯 Mouse Hazard ACTIVE — Move cursor across track to test AI evasion!' );

		} else {

			if ( this.mouseObstacleGroup ) {

				this.mouseObstacleGroup.visible = false;

			}

			if ( this.mouseObstacleHitMesh ) {

				this.allObstacles = this.allObstacles.filter( ( m ) => m !== this.mouseObstacleHitMesh );

			}

			this.showToast( 'Mouse Hazard Test Disabled' );

		}

		this.updateMouseObstacleUI();

	}

	setMouseObstacleRadius( r ) {

		this.mouseObstacleRadius = THREE.MathUtils.clamp( r, 0.4, 3.0 );
		if ( this.mouseObstacleHitMesh ) {

			this.mouseObstacleHitMesh.geometry.dispose();
			const hitGeom = new THREE.CylinderGeometry( this.mouseObstacleRadius, this.mouseObstacleRadius, 2.4, 16 );
			hitGeom.translate( 0, 1.2, 0 );
			this.mouseObstacleHitMesh.geometry = hitGeom;

		}
		if ( this.mouseObstacleRing ) {

			const s = this.mouseObstacleRadius;
			this.mouseObstacleRing.scale.set( s, 1, s );
			this.mouseObstacleOuterRing.scale.set( s, 1, s );

		}

	}

	placeMouseObstacleInFrontOfCar( vehicle, forwardDistance = 7.0 ) {

		const v = vehicle || this.lastVehicle;
		if ( ! v ) return;

		_forward.set( 0, 0, 1 ).applyQuaternion( v.container.quaternion );
		_forward.y = 0;
		_forward.normalize();

		const x = v.spherePos.x + _forward.x * forwardDistance;
		const z = v.spherePos.z + _forward.z * forwardDistance;

		this.setMouseObstaclePosition( x, z );
		if ( ! this.mouseObstacleActive ) {

			this.toggleMouseObstacle( true );

		}

	}

	showToast( msg ) {

		if ( ! this.toastBanner ) return;
		this.toastBanner.textContent = msg;
		this.toastBanner.classList.add( 'show' );

		clearTimeout( this.toastTimeout );
		this.toastTimeout = setTimeout( () => {

			if ( this.toastBanner ) {

				this.toastBanner.classList.remove( 'show' );

			}

		}, 2600 );

	}

	updateMouseObstacleUI() {

		if ( this.mouseHazardBtn ) {

			this.mouseHazardBtn.classList.toggle( 'hazard-active', this.mouseObstacleActive );
			const textEl = this.mouseHazardBtn.querySelector( '#ai-hazard-text' );
			if ( textEl ) {

				textEl.textContent = this.mouseObstacleActive ? 'Mouse Hazard: ON' : 'Mouse Hazard';

			}

		}

		if ( this.tunerPanel ) {

			const chk = this.tunerPanel.querySelector( '#ai-chk-mouse-hazard' );
			if ( chk ) chk.checked = this.mouseObstacleActive;

		}

	}

	buildVisualSensors() {

		if ( ! this.scene ) return;

		const lineCount = LIDAR_RAYS.length;
		const positions = new Float32Array( lineCount * 2 * 3 );
		const colors = new Float32Array( lineCount * 2 * 3 );

		for ( let i = 0; i < lineCount; i ++ ) {

			colors[ i * 6 + 0 ] = 0.2;
			colors[ i * 6 + 1 ] = 0.9;
			colors[ i * 6 + 2 ] = 0.4;
			colors[ i * 6 + 3 ] = 0.2;
			colors[ i * 6 + 4 ] = 0.9;
			colors[ i * 6 + 5 ] = 0.4;

		}

		const geom = new THREE.BufferGeometry();
		geom.setAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
		geom.setAttribute( 'color', new THREE.BufferAttribute( colors, 3 ) );

		const mat = new THREE.LineBasicMaterial( {
			vertexColors: true,
			transparent: true,
			opacity: 0.85,
			depthWrite: false,
		} );

		this.sensorLinesMesh = new THREE.LineSegments( geom, mat );
		this.sensorLinesMesh.frustumCulled = false;
		this.sensorLinesMesh.visible = false;
		this.scene.add( this.sensorLinesMesh );

		// Target Waypoint Ring on spline
		const ringGeom = new THREE.RingGeometry( 0.35, 0.55, 24 );
		ringGeom.rotateX( - Math.PI / 2 );
		const ringMat = new THREE.MeshBasicMaterial( {
			color: 0x38bdf8,
			transparent: true,
			opacity: 0.9,
			side: THREE.DoubleSide,
			depthWrite: false,
		} );
		this.targetRing = new THREE.Mesh( ringGeom, ringMat );
		this.targetRing.visible = false;
		this.scene.add( this.targetRing );

	}

	buildHUD() {

		const style = document.createElement( 'style' );
		style.textContent = `
			#ai-pilot-bar {
				position: absolute;
				top: 14px;
				left: 50%;
				transform: translateX(-50%);
				display: flex;
				align-items: center;
				gap: 8px;
				z-index: 30;
				user-select: none;
			}
			.ai-pill-btn {
				color: #0f172a;
				font: 600 13px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
				background: rgba(255, 255, 255, 0.94);
				padding: 7px 16px;
				border-radius: 999px;
				border: 1px solid rgba(0, 0, 0, 0.08);
				box-shadow: 0 4px 14px rgba(0, 0, 0, 0.12);
				backdrop-filter: blur(12px);
				-webkit-backdrop-filter: blur(12px);
				cursor: pointer;
				display: flex;
				align-items: center;
				gap: 7px;
				transition: all 0.2s cubic-bezier(0.16, 1, 0.3, 1);
			}
			.ai-pill-btn:hover {
				background: #ffffff;
				transform: scale(1.03);
			}
			.ai-pill-btn.active {
				background: #0f172a;
				color: #f8fafc;
				border-color: rgba(34, 197, 94, 0.5);
				box-shadow: 0 4px 18px rgba(34, 197, 94, 0.35);
			}
			.ai-pill-btn.hazard-active {
				background: #450a0a;
				color: #fecaca;
				border-color: rgba(239, 68, 68, 0.8);
				box-shadow: 0 4px 18px rgba(239, 68, 68, 0.45);
			}
			.ai-pill-btn.hazard-active #ai-hazard-icon {
				display: inline-block;
				animation: ai-hazard-pulse 0.9s infinite ease-in-out;
			}
			@keyframes ai-hazard-pulse {
				0%, 100% { transform: scale(1); }
				50% { transform: scale(1.3); }
			}
			#ai-toast-banner {
				position: absolute;
				top: 58px;
				left: 50%;
				transform: translateX(-50%) translateY(-10px);
				background: rgba(15, 23, 42, 0.94);
				color: #f8fafc;
				font: 500 12px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
				padding: 8px 18px;
				border-radius: 999px;
				border: 1px solid rgba(239, 68, 68, 0.5);
				box-shadow: 0 8px 24px rgba(0, 0, 0, 0.35);
				backdrop-filter: blur(12px);
				z-index: 32;
				pointer-events: none;
				opacity: 0;
				transition: opacity 0.25s ease, transform 0.25s ease;
			}
			#ai-toast-banner.show {
				opacity: 1;
				transform: translateX(-50%) translateY(0);
			}
			#ai-indicator-dot {
				width: 8px;
				height: 8px;
				border-radius: 50%;
				background: #94a3b8;
				transition: background 0.2s;
			}
			.ai-pill-btn.active #ai-indicator-dot {
				background: #22c55e;
				box-shadow: 0 0 10px #22c55e;
				animation: ai-pulse 1.4s infinite ease-in-out;
			}
			@keyframes ai-pulse {
				0%, 100% { opacity: 1; transform: scale(1); }
				50% { opacity: 0.4; transform: scale(0.8); }
			}
			.ai-k-badge {
				font-size: 11px;
				opacity: 0.6;
				margin-left: 2px;
				font-weight: 500;
			}

			/* Floating Hacker Studio & Tuner */
			#ai-tuner-panel {
				position: absolute;
				top: 64px;
				left: 18px;
				width: 330px;
				background: rgba(15, 23, 42, 0.95);
				color: #e2e8f0;
				font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
				font-size: 12px;
				border-radius: 14px;
				border: 1px solid rgba(255, 255, 255, 0.12);
				box-shadow: 0 12px 36px rgba(0, 0, 0, 0.5);
				backdrop-filter: blur(16px);
				-webkit-backdrop-filter: blur(16px);
				z-index: 35;
				display: none;
				flex-direction: column;
				overflow: hidden;
				user-select: none;
			}
			#ai-tuner-panel.open { display: flex; }
			.ai-panel-header {
				padding: 10px 14px;
				background: rgba(255, 255, 255, 0.05);
				border-bottom: 1px solid rgba(255, 255, 255, 0.08);
				display: flex;
				justify-content: space-between;
				align-items: center;
				font-weight: 700;
				letter-spacing: 0.04em;
				color: #38bdf8;
			}
			.ai-panel-body {
				padding: 12px 14px;
				max-height: 460px;
				overflow-y: auto;
				display: flex;
				flex-direction: column;
				gap: 12px;
			}
			.ai-panel-tabs {
				display: flex;
				gap: 6px;
				border-bottom: 1px solid rgba(255, 255, 255, 0.08);
				padding: 0 14px;
				background: rgba(0, 0, 0, 0.2);
			}
			.ai-tab-btn {
				background: transparent;
				border: none;
				color: #94a3b8;
				padding: 8px 10px;
				font-size: 11px;
				font-weight: 600;
				cursor: pointer;
				border-bottom: 2px solid transparent;
				transition: all 0.2s;
			}
			.ai-tab-btn.active {
				color: #38bdf8;
				border-bottom-color: #38bdf8;
			}
			.ai-control-row {
				display: flex;
				flex-direction: column;
				gap: 4px;
			}
			.ai-control-label {
				display: flex;
				justify-content: space-between;
				color: #cbd5e1;
				font-size: 11px;
			}
			.ai-slider {
				width: 100%;
				height: 5px;
				accent-color: #38bdf8;
				cursor: pointer;
			}
			.ai-preset-grid {
				display: grid;
				grid-template-columns: 1fr 1fr;
				gap: 6px;
			}
			.ai-preset-btn {
				background: rgba(255, 255, 255, 0.06);
				border: 1px solid rgba(255, 255, 255, 0.1);
				color: #f1f5f9;
				padding: 6px 8px;
				border-radius: 6px;
				font-size: 11px;
				cursor: pointer;
				text-align: left;
				transition: all 0.15s;
			}
			.ai-preset-btn:hover {
				background: rgba(56, 189, 248, 0.15);
				border-color: #38bdf8;
			}
			.ai-preset-btn.active {
				background: #38bdf8;
				color: #0f172a;
				font-weight: 700;
			}
			.ai-telemetry-grid {
				display: grid;
				grid-template-columns: 1fr 1fr;
				gap: 6px;
				background: rgba(0, 0, 0, 0.3);
				padding: 8px;
				border-radius: 8px;
			}
			.ai-metric-item {
				display: flex;
				flex-direction: column;
			}
			.ai-metric-label {
				font-size: 10px;
				color: #64748b;
				text-transform: uppercase;
			}
			.ai-metric-val {
				font-size: 12px;
				font-weight: 700;
				color: #f8fafc;
			}
			.ai-code-editor {
				width: 100%;
				height: 150px;
				background: #090d16;
				color: #7dd3fc;
				font-family: inherit;
				font-size: 11px;
				padding: 8px;
				border: 1px solid rgba(255, 255, 255, 0.12);
				border-radius: 6px;
				resize: vertical;
				outline: none;
				box-sizing: border-box;
			}
			.ai-run-btn {
				background: #22c55e;
				color: #022c22;
				border: none;
				font-weight: 700;
				padding: 7px;
				border-radius: 6px;
				cursor: pointer;
				transition: all 0.2s;
			}
			.ai-run-btn:hover {
				background: #4ade80;
			}
		`;
		document.head.appendChild( style );

		const bar = document.createElement( 'div' );
		bar.id = 'ai-pilot-bar';
		bar.innerHTML = `
			<button id="ai-pilot-toggle" class="ai-pill-btn" title="Toggle Auto-Pilot [C]">
				<span id="ai-indicator-dot"></span>
				<span id="ai-toggle-text">AI Auto-Pilot</span>
				<span class="ai-k-badge">[C]</span>
			</button>
			<button id="ai-mouse-hazard-toggle" class="ai-pill-btn" title="Toggle Mouse Cursor Obstacle Hazard [M]">
				<span id="ai-hazard-icon">🎯</span>
				<span id="ai-hazard-text">Mouse Hazard</span>
				<span class="ai-k-badge">[M]</span>
			</button>
			<button id="ai-tuner-toggle" class="ai-pill-btn" title="Open AI Tuning & Hacker Deck [H]">
				<span>⚙️ AI Deck</span>
				<span class="ai-k-badge">[H]</span>
			</button>
		`;

		document.body.appendChild( bar );

		const toast = document.createElement( 'div' );
		toast.id = 'ai-toast-banner';
		document.body.appendChild( toast );
		this.toastBanner = toast;

		const toggleBtn = bar.querySelector( '#ai-pilot-toggle' );
		const mouseHazardBtn = bar.querySelector( '#ai-mouse-hazard-toggle' );
		const tunerBtn = bar.querySelector( '#ai-tuner-toggle' );

		toggleBtn.addEventListener( 'click', () => this.toggle() );
		mouseHazardBtn.addEventListener( 'click', () => this.toggleMouseObstacle() );
		tunerBtn.addEventListener( 'click', () => this.toggleTuner() );

		window.addEventListener( 'keydown', ( e ) => {

			if ( e.target.tagName === 'TEXTAREA' || e.target.tagName === 'INPUT' ) return;

			if ( e.code === 'KeyC' && ! e.repeat && ! e.ctrlKey && ! e.metaKey ) {

				this.toggle();

			} else if ( e.code === 'KeyM' && ! e.repeat && ! e.ctrlKey && ! e.metaKey ) {

				this.toggleMouseObstacle();

			} else if ( e.code === 'KeyH' && ! e.repeat && ! e.ctrlKey && ! e.metaKey ) {

				this.toggleTuner();

			}

		} );

		this.toggleBtn = toggleBtn;
		this.mouseHazardBtn = mouseHazardBtn;
		this.tunerBtn = tunerBtn;

		this.buildTunerPanel();

	}

	buildTunerPanel() {

		const panel = document.createElement( 'div' );
		panel.id = 'ai-tuner-panel';
		panel.innerHTML = `
			<div class="ai-panel-header">
				<span>⚡ AUTOPILOT HACKER DECK</span>
				<span style="font-size:10px; color:#64748b;">window.autopilot</span>
			</div>
			<div class="ai-panel-tabs">
				<button class="ai-tab-btn active" data-tab="tuning">Tuning</button>
				<button class="ai-tab-btn" data-tab="telemetry">LiDAR Sensors</button>
				<button class="ai-tab-btn" data-tab="code">Custom JS</button>
			</div>
			<div class="ai-panel-body" id="ai-tab-tuning">
				<div class="ai-control-row">
					<span style="font-size:10px; color:#94a3b8; text-transform:uppercase;">Driving Presets</span>
					<div class="ai-preset-grid">
						<button class="ai-preset-btn active" data-preset="racing">🏎️ Racing</button>
						<button class="ai-preset-btn" data-preset="safe">🛡️ Safe</button>
						<button class="ai-preset-btn" data-preset="drift">💨 Drift</button>
						<button class="ai-preset-btn" data-preset="speed">⚡ Speed</button>
					</div>
				</div>
				<div class="ai-control-row">
					<div class="ai-control-label">
						<span>Steer Gain (Kp)</span>
						<span id="ai-val-steer">${this.config.steerGain.toFixed( 1 )}</span>
					</div>
					<input type="range" class="ai-slider" id="ai-sl-steer" min="1.0" max="6.0" step="0.1" value="${this.config.steerGain}">
				</div>
				<div class="ai-control-row">
					<div class="ai-control-label">
						<span>Lookahead Base (m)</span>
						<span id="ai-val-lookahead">${this.config.lookaheadBase.toFixed( 1 )}</span>
					</div>
					<input type="range" class="ai-slider" id="ai-sl-lookahead" min="2.0" max="8.0" step="0.2" value="${this.config.lookaheadBase}">
				</div>
				<div class="ai-control-row">
					<div class="ai-control-label">
						<span>Wall Repulsion Gain</span>
						<span id="ai-val-wall">${this.config.wallAvoidance.toFixed( 1 )}</span>
					</div>
					<input type="range" class="ai-slider" id="ai-sl-wall" min="0.0" max="4.0" step="0.1" value="${this.config.wallAvoidance}">
				</div>
				<div class="ai-control-row">
					<div class="ai-control-label">
						<span>Corner Braking Factor</span>
						<span id="ai-val-brake">${this.config.cornerBraking.toFixed( 2 )}</span>
					</div>
					<input type="range" class="ai-slider" id="ai-sl-brake" min="0.0" max="2.0" step="0.05" value="${this.config.cornerBraking}">
				</div>
				<div class="ai-control-row">
					<div class="ai-control-label">
						<span>Wall Safety Margin (m)</span>
						<span id="ai-val-margin">${this.config.wallSafetyMargin.toFixed( 1 )}</span>
					</div>
					<input type="range" class="ai-slider" id="ai-sl-margin" min="1.0" max="3.5" step="0.1" value="${this.config.wallSafetyMargin}">
				</div>
				<div class="ai-control-row" style="flex-direction:row; align-items:center; justify-content:space-between; margin-top:4px;">
					<span>Show 3D LiDAR Lasers</span>
					<input type="checkbox" id="ai-chk-sensors" ${this.config.showSensors ? 'checked' : ''}>
				</div>
				<div class="ai-control-row" style="background:rgba(239, 68, 68, 0.08); border:1px solid rgba(239, 68, 68, 0.25); border-radius:10px; padding:10px; gap:8px; margin-top:6px;">
					<div style="display:flex; justify-content:space-between; align-items:center;">
						<span style="font-weight:700; color:#f87171; font-size:12px;">🎯 Mouse Cursor Hazard</span>
						<input type="checkbox" id="ai-chk-mouse-hazard" ${this.mouseObstacleActive ? 'checked' : ''}>
					</div>
					<div style="font-size:10px; color:#94a3b8; line-height:1.4;">
						Cursor projects a live 3D obstacle hazard onto the track plane. The AI senses it via 9-ray LiDAR and dynamically steers to evade or emergency brakes.
					</div>
					<div class="ai-control-row">
						<div class="ai-control-label">
							<span>Hazard Radius (m)</span>
							<span id="ai-val-hazard-rad">${this.mouseObstacleRadius.toFixed( 1 )}m</span>
						</div>
						<input type="range" class="ai-slider" id="ai-sl-hazard-rad" min="0.5" max="2.5" step="0.1" value="${this.mouseObstacleRadius}">
					</div>
					<div style="display:flex; gap:6px;">
						<button class="ai-preset-btn" id="ai-btn-place-front" style="flex:1; padding:6px 8px; font-size:10px;">📍 Drop Ahead of Car</button>
					</div>
				</div>
			</div>

			<div class="ai-panel-body" id="ai-tab-telemetry" style="display:none;">
				<div class="ai-telemetry-grid">
					<div class="ai-metric-item">
						<span class="ai-metric-label">AI Status</span>
						<span class="ai-metric-val" id="ai-m-status" style="color:#38bdf8;">OFF</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">True Speed</span>
						<span class="ai-metric-val" id="ai-m-speed">0.0 m/s</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Steer</span>
						<span class="ai-metric-val" id="ai-m-steer">0.00</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Throttle</span>
						<span class="ai-metric-val" id="ai-m-throttle">0%</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Center Offset</span>
						<span class="ai-metric-val" id="ai-m-offset">0.0m</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Nearest Wall</span>
						<span class="ai-metric-val" id="ai-m-nearest" style="color:#22c55e;">25.0m</span>
					</div>
					<div class="ai-metric-item" style="grid-column: span 2;">
						<span class="ai-metric-label">Mouse Cursor Hazard</span>
						<span class="ai-metric-val" id="ai-m-mouse-hazard" style="color:#64748b;">DISABLED [Press M]</span>
					</div>
				</div>
				<div style="font-size:11px; color:#94a3b8; margin-top:4px;">Ray Distances to Walls (m):</div>
				<div class="ai-telemetry-grid">
					<div class="ai-metric-item">
						<span class="ai-metric-label">Front Radar</span>
						<span class="ai-metric-val" id="ai-m-rfwd">--</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Fwd L / R</span>
						<span class="ai-metric-val" id="ai-m-rfwdlr">-- / --</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Diag L / R</span>
						<span class="ai-metric-val" id="ai-m-rdiaglr">-- / --</span>
					</div>
					<div class="ai-metric-item">
						<span class="ai-metric-label">Side L / R</span>
						<span class="ai-metric-val" id="ai-m-rsidelr">-- / --</span>
					</div>
				</div>
			</div>

			<div class="ai-panel-body" id="ai-tab-code" style="display:none;">
				<div style="font-size:10px; color:#94a3b8;">
					Write custom JS driving logic! (Return <code>{ steer, throttle }</code>):
				</div>
				<textarea class="ai-code-editor" id="ai-code-input" spellcheck="false">// (sensors, telemetry) => { steer, throttle }
// Note: In vehicle physics, steer < 0 turns RIGHT, steer > 0 turns LEFT.
const { fwdFar, fwdLeft, fwdRight, diagLeft, diagRight, sideLeft, sideRight, trackOffset } = sensors;
const { trueSpeed } = telemetry;

// 1. Centerline tracking (steer towards road center)
let steer = ( trackOffset > 0 ? 1 : -1 ) * Math.min( Math.abs( trackOffset ) * 0.4, 0.7 );

// 2. Proactive LiDAR boundary repulsion
if ( sideLeft < 2.2 ) steer -= ( 2.2 - sideLeft ) * 0.8; // Turn right away from left wall
if ( sideRight < 2.2 ) steer += ( 2.2 - sideRight ) * 0.8; // Turn left away from right wall
if ( diagLeft < 2.6 ) steer -= ( 2.6 - diagLeft ) * 0.6;
if ( diagRight < 2.6 ) steer += ( 2.6 - diagRight ) * 0.6;

// 3. Wall proximity speed control
const frontWall = Math.min( fwdFar, fwdLeft, fwdRight );
let throttle = 1.0;
if ( frontWall < 5.5 ) {
    throttle = Math.max( 0.35, ( frontWall - 1.5 ) / 4.0 );
}

return { steer, throttle };</textarea>
				<button class="ai-run-btn" id="ai-code-run">▶ Apply Live Code</button>
				<div id="ai-code-status" style="font-size:10px; color:#22c55e;">Ready to inject</div>
			</div>
		`;

		document.body.appendChild( panel );
		this.tunerPanel = panel;

		panel.querySelectorAll( '.ai-tab-btn' ).forEach( ( btn ) => {

			btn.addEventListener( 'click', () => {

				panel.querySelectorAll( '.ai-tab-btn' ).forEach( ( b ) => b.classList.remove( 'active' ) );
				panel.querySelectorAll( '.ai-panel-body' ).forEach( ( b ) => b.style.display = 'none' );
				btn.classList.add( 'active' );
				panel.querySelector( `#ai-tab-${ btn.dataset.tab }` ).style.display = 'flex';

			} );

		} );

		const bindSlider = ( id, key, labelId, format = ( v ) => v.toFixed( 1 ) ) => {

			const el = panel.querySelector( id );
			const valEl = panel.querySelector( labelId );
			el.addEventListener( 'input', () => {

				const v = parseFloat( el.value );
				this.config[ key ] = v;
				valEl.textContent = format( v );

			} );

		};

		bindSlider( '#ai-sl-steer', 'steerGain', '#ai-val-steer' );
		bindSlider( '#ai-sl-lookahead', 'lookaheadBase', '#ai-val-lookahead' );
		bindSlider( '#ai-sl-wall', 'wallAvoidance', '#ai-val-wall' );
		bindSlider( '#ai-sl-brake', 'cornerBraking', '#ai-val-brake', ( v ) => v.toFixed( 2 ) );
		bindSlider( '#ai-sl-margin', 'wallSafetyMargin', '#ai-val-margin' );

		panel.querySelector( '#ai-chk-sensors' ).addEventListener( 'change', ( e ) => {

			this.config.showSensors = e.target.checked;
			if ( this.sensorLinesMesh ) this.sensorLinesMesh.visible = this.enabled && this.config.showSensors;
			if ( this.targetRing ) this.targetRing.visible = this.enabled && this.config.showSensors;

		} );

		const mouseHazardChk = panel.querySelector( '#ai-chk-mouse-hazard' );
		if ( mouseHazardChk ) {

			mouseHazardChk.addEventListener( 'change', ( e ) => {

				this.toggleMouseObstacle( e.target.checked );

			} );

		}

		const hazardRadSl = panel.querySelector( '#ai-sl-hazard-rad' );
		const hazardRadVal = panel.querySelector( '#ai-val-hazard-rad' );
		if ( hazardRadSl && hazardRadVal ) {

			hazardRadSl.addEventListener( 'input', () => {

				const r = parseFloat( hazardRadSl.value );
				this.setMouseObstacleRadius( r );
				hazardRadVal.textContent = `${ r.toFixed( 1 ) }m`;

			} );

		}

		const btnPlaceFront = panel.querySelector( '#ai-btn-place-front' );
		if ( btnPlaceFront ) {

			btnPlaceFront.addEventListener( 'click', () => {

				this.placeMouseObstacleInFrontOfCar( this.lastVehicle, 7.0 );

			} );

		}

		panel.querySelectorAll( '.ai-preset-btn' ).forEach( ( btn ) => {

			btn.addEventListener( 'click', () => {

				this.applyPreset( btn.dataset.preset );

			} );

		} );

		panel.querySelector( '#ai-code-run' ).addEventListener( 'click', () => {

			const code = panel.querySelector( '#ai-code-input' ).value;
			this.setCustomCode( code );

		} );

	}

	applyPreset( presetKey ) {

		const p = AI_PRESETS[ presetKey ];
		if ( ! p ) return;

		this.config.activePreset = presetKey;
		this.config.steerGain = p.steerGain;
		this.config.lookaheadBase = p.lookaheadBase;
		this.config.cornerBraking = p.cornerBraking;
		this.config.wallAvoidance = p.wallAvoidance;
		this.config.maxThrottle = p.maxThrottle;
		this.config.targetSpeed = p.targetSpeed || 1.15;
		this.config.wallSafetyMargin = p.wallSafetyMargin;
		this.config.useCustomCode = false;

		if ( this.tunerPanel ) {

			this.tunerPanel.querySelectorAll( '.ai-preset-btn' ).forEach( ( b ) => {

				b.classList.toggle( 'active', b.dataset.preset === presetKey );

			} );

			this.tunerPanel.querySelector( '#ai-sl-steer' ).value = p.steerGain;
			this.tunerPanel.querySelector( '#ai-val-steer' ).textContent = p.steerGain.toFixed( 1 );
			this.tunerPanel.querySelector( '#ai-sl-lookahead' ).value = p.lookaheadBase;
			this.tunerPanel.querySelector( '#ai-val-lookahead' ).textContent = p.lookaheadBase.toFixed( 1 );
			this.tunerPanel.querySelector( '#ai-sl-wall' ).value = p.wallAvoidance;
			this.tunerPanel.querySelector( '#ai-val-wall' ).textContent = p.wallAvoidance.toFixed( 1 );
			this.tunerPanel.querySelector( '#ai-sl-brake' ).value = p.cornerBraking;
			this.tunerPanel.querySelector( '#ai-val-brake' ).textContent = p.cornerBraking.toFixed( 2 );
			this.tunerPanel.querySelector( '#ai-sl-margin' ).value = p.wallSafetyMargin;
			this.tunerPanel.querySelector( '#ai-val-margin' ).textContent = p.wallSafetyMargin.toFixed( 1 );

		}

	}

	setCustomCode( codeString ) {

		try {

			const fn = new Function( 'sensors', 'telemetry', `
				"use strict";
				${ codeString }
			` );

			fn( this.sensors, this.telemetry );

			this.customLogicFunction = fn;
			this.config.useCustomCode = true;

			const statusEl = this.tunerPanel.querySelector( '#ai-code-status' );
			statusEl.textContent = '✓ Active & Running Live!';
			statusEl.style.color = '#22c55e';

		} catch ( err ) {

			console.error( 'Autopilot custom code compilation error:', err );
			const statusEl = this.tunerPanel.querySelector( '#ai-code-status' );
			statusEl.textContent = `Error: ${ err.message }`;
			statusEl.style.color = '#ef4444';

		}

	}

	toggle( forceState ) {

		this.enabled = ( forceState !== undefined ) ? forceState : ! this.enabled;

		if ( this.toggleBtn ) {

			if ( this.enabled ) {

				this.toggleBtn.classList.add( 'active' );
				this.toggleBtn.querySelector( '#ai-toggle-text' ).textContent = 'AI Pilot: ON';

			} else {

				this.toggleBtn.classList.remove( 'active' );
				this.toggleBtn.querySelector( '#ai-toggle-text' ).textContent = 'AI Auto-Pilot';

			}

		}

		if ( this.sensorLinesMesh ) this.sensorLinesMesh.visible = this.enabled && this.config.showSensors;
		if ( this.targetRing ) this.targetRing.visible = this.enabled && this.config.showSensors;

		this.isRecovering = false;
		this.recoveryTimer = 0;
		this.stuckWatchTimer = 0;
		this.smoothedSteer = 0;
		this.needsGlobalSplineLocate = true;

	}

	toggleTuner( forceState ) {

		if ( ! this.tunerPanel ) return;
		const isOpen = ( forceState !== undefined ) ? forceState : ! this.tunerPanel.classList.contains( 'open' );

		if ( isOpen ) {

			this.tunerPanel.classList.add( 'open' );
			this.tunerBtn.classList.add( 'active' );

		} else {

			this.tunerPanel.classList.remove( 'open' );
			this.tunerBtn.classList.remove( 'active' );

		}

	}

	/**
	 * Main real-time perception and control tick.
	 */
	update( dt, vehicle, humanInput ) {

		this.lastVehicle = vehicle;

		// Animate mouse hazard visual beacon and ground radar pulse
		if ( this.mouseObstacleGroup && this.mouseObstacleActive ) {

			if ( this.mouseObstacleTopper ) {

				this.mouseObstacleTopper.rotation.y += dt * 3.5;
				this.mouseObstacleTopper.rotation.x += dt * 1.8;

			}
			if ( this.mouseObstacleRing ) {

				const pulse = ( 1.0 + Math.sin( performance.now() * 0.008 ) * 0.12 ) * this.mouseObstacleRadius;
				this.mouseObstacleRing.scale.set( pulse, 1, pulse );

			}

		}

		if ( ! this.enabled || ! this.spline || this.samples.length === 0 ) return humanInput;

		// Manual player override disengages autopilot
		const humanActive = humanInput.touchActive ||
			( Math.abs( humanInput.x ) > 0.65 || Math.abs( humanInput.z ) > 0.65 );

		if ( humanActive ) {

			this.toggle( false );
			return humanInput;

		}

		_carPos.copy( vehicle.container.position );
		_forward.set( 0, 0, 1 ).applyQuaternion( vehicle.container.quaternion );
		_forward.y = 0;
		_forward.normalize();

		_right.set( 1, 0, 0 ).applyQuaternion( vehicle.container.quaternion );
		_right.y = 0;
		_right.normalize();

		// Actual physical speed in world units/sec
		const trueSpeed = vehicle.modelVelocity.length();
		this.telemetry.trueSpeed = trueSpeed;

		// ─── 1. Real-Time 3D Raycasting against Track Wall Geometry ─────────────────
		_rayOrigin.copy( _carPos );
		_rayOrigin.y = 0.45; // Ray origin at car bumper height

		const linePositions = this.sensorLinesMesh ? this.sensorLinesMesh.geometry.attributes.position.array : null;
		const lineColors = this.sensorLinesMesh ? this.sensorLinesMesh.geometry.attributes.color.array : null;

		let nearestDist = Infinity;

		for ( let i = 0; i < LIDAR_RAYS.length; i ++ ) {

			const cfg = LIDAR_RAYS[ i ];
			const rad = THREE.MathUtils.degToRad( cfg.angle );

			// Direction of this ray relative to vehicle forward vector
			_rayDir.copy( _forward ).applyAxisAngle( _upAxis, rad ).normalize();

			this.raycaster.set( _rayOrigin, _rayDir );
			this.raycaster.far = cfg.range;

			let hitDist = cfg.range;
			_rayEnd.copy( _rayOrigin ).addScaledVector( _rayDir, cfg.range );

			if ( this.allObstacles.length > 0 ) {

				const intersects = this.raycaster.intersectObjects( this.allObstacles, false );
				if ( intersects.length > 0 && intersects[ 0 ].distance < cfg.range ) {

					hitDist = intersects[ 0 ].distance;
					_rayEnd.copy( intersects[ 0 ].point );

				}

			}

			this.sensors[ cfg.id ] = hitDist;
			if ( hitDist < nearestDist ) nearestDist = hitDist;

			// Update 3D visual laser line vertices and dynamic alert colors
			if ( linePositions && lineColors && this.config.showSensors ) {

				const idx = i * 6;
				linePositions[ idx + 0 ] = _rayOrigin.x;
				linePositions[ idx + 1 ] = _rayOrigin.y;
				linePositions[ idx + 2 ] = _rayOrigin.z;

				linePositions[ idx + 3 ] = _rayEnd.x;
				linePositions[ idx + 4 ] = _rayEnd.y;
				linePositions[ idx + 5 ] = _rayEnd.z;

				// Colors: Cyan/Green (>3.0m) -> Amber (1.8 - 3.0m) -> Danger Red (<1.8m)
				let r = 0.2, g = 0.85, b = 0.5;
				if ( hitDist < 1.8 ) {

					r = 0.95; g = 0.2; b = 0.2; // Red

				} else if ( hitDist < 3.0 ) {

					r = 0.95; g = 0.75; b = 0.15; // Amber

				}

				lineColors[ idx + 0 ] = r; lineColors[ idx + 1 ] = g; lineColors[ idx + 2 ] = b;
				lineColors[ idx + 3 ] = r; lineColors[ idx + 4 ] = g; lineColors[ idx + 5 ] = b;

			}

		}

		this.sensors.nearestWallDist = nearestDist;

		if ( linePositions && this.config.showSensors && this.sensorLinesMesh.visible ) {

			this.sensorLinesMesh.geometry.attributes.position.needsUpdate = true;
			this.sensorLinesMesh.geometry.attributes.color.needsUpdate = true;

		}

		// ─── 2. Real-Time Obstacle & Spatial Corridor Analysis ─────────────────────
		const posMoveDelta = _carPos.distanceTo( this.lastPosition );
		this.lastPosition.copy( _carPos );

		// Forward threat: closest object in forward cone
		const frontThreat = Math.min( this.sensors.fwdFar, this.sensors.fwdLeft * 1.15, this.sensors.fwdRight * 1.15 );
		const leftClearance = Math.min( this.sensors.fwdLeft, this.sensors.diagLeft, this.sensors.sideLeft );
		const rightClearance = Math.min( this.sensors.fwdRight, this.sensors.diagRight, this.sensors.sideRight );
		const rearClear = Math.min( this.sensors.rearLeft, this.sensors.rearRight );

		// Time-To-Collision (TTC) based on true physical velocity
		const forwardSpeed = Math.max( 0.2, vehicle.modelVelocity.dot( _forward ) );
		const timeToCollision = frontThreat / forwardSpeed;

		// Pinned / Stuck Detector (3-Point Recovery Trigger)
		// Detect when vehicle is stopped or making no forward progress despite autopilot being active
		const isImmobilized = ( trueSpeed < 0.35 && posMoveDelta < 0.05 );
		const isThreatPinned = ( frontThreat < 1.8 && trueSpeed < 0.50 && posMoveDelta < 0.08 );
		const isWallPinned = ( this.sensors.nearestWallDist < 0.95 && trueSpeed < 0.30 );
		const isStuckCandidate = isImmobilized || isThreatPinned || isWallPinned;

		if ( isStuckCandidate && this.recoveryPhase === 0 ) {

			this.stuckWatchTimer += dt;
			this.totalStuckDuration += dt;

			if ( this.stuckWatchTimer > 0.40 ) {

				this.recoveryCount ++;

				// If car has been stuck across multiple recovery cycles (> 3 attempts or > 3.0s total), trigger emergency track realignment
				if ( this.recoveryCount > 3 || this.totalStuckDuration > 3.0 ) {

					this.telemetry.status = 'RECOVERY: EMERGENCY TRACK REALIGN';
					this.decisionState = 'RECOVERY_REALIGN';
					this.realignToTrackSpline( vehicle );
					this.recoveryPhase = 0;
					this.stuckWatchTimer = 0;
					this.totalStuckDuration = 0;
					this.recoveryCount = 0;
					this.needsGlobalSplineLocate = true;
					this.updateTelemetryHUD( 0.60, 0 );
					return { x: 0, z: 0.60, touchActive: false };

				}

				// Phase 1: Reversing with counter-angle to swing nose clear
				this.recoveryPhase = 1;
				this.recoveryTimer = 0.90;
				// Evaluate side clearance to steer towards the wider opening
				const leftRoom = this.sensors.sideLeft + this.sensors.diagLeft * 0.8;
				const rightRoom = this.sensors.sideRight + this.sensors.diagRight * 0.8;
				this.recoverySteer = leftRoom >= rightRoom ? - 1.0 : 1.0;

			}

		} else if ( this.recoveryPhase === 0 ) {

			this.stuckWatchTimer = Math.max( 0, this.stuckWatchTimer - dt * 2.0 );
			if ( trueSpeed > 0.9 && posMoveDelta > 0.12 ) {

				this.totalStuckDuration = 0;
				this.recoveryCount = 0;

			}

		}

		// ─── Multi-Phase 3-Point Unstuck Recovery Execution ────────────────────────
		if ( this.recoveryPhase === 1 ) {

			// Phase 1: Reversing with full reverse torque and counter-angle to swing nose clear
			this.recoveryTimer -= dt;
			this.telemetry.status = 'RECOVERY: REVERSING';
			this.decisionState = 'RECOVERY_REVERSE';

			if ( this.recoveryTimer <= 0 || frontThreat > 3.2 || rearClear < 0.9 ) {

				// Transition to Phase 2: Forward pivot into open corridor
				this.recoveryPhase = 2;
				this.recoveryTimer = 0.55;
				this.recoverySteer = - this.recoverySteer * 0.9; // Steer into open space

			} else {

				this.updateTelemetryHUD( - 0.95, this.recoverySteer );
				return { x: this.recoverySteer, z: - 0.95, touchActive: false };

			}

		}

		if ( this.recoveryPhase === 2 ) {

			// Phase 2: Forward swing into open racing line with positive drive
			this.recoveryTimer -= dt;
			this.telemetry.status = 'RECOVERY: PIVOTING FORWARD';
			this.decisionState = 'RECOVERY_PIVOT';

			if ( this.recoveryTimer <= 0 || ( frontThreat > 3.5 && trueSpeed > 0.9 ) ) {

				this.recoveryPhase = 0;
				this.stuckWatchTimer = 0;
				this.needsGlobalSplineLocate = true;

			} else {

				this.updateTelemetryHUD( 0.85, this.recoverySteer );
				return { x: this.recoverySteer, z: 0.85, touchActive: false };

			}

		}

		// ─── 3. Spline Tracking & Pure Pursuit Target ──────────────────────────────
		// Find closest point on track centerline spline
		let bestIdx = this.closestSampleIdx;
		let minDistSq = Infinity;

		if ( this.needsGlobalSplineLocate ) {

			// Full circuit scan on activation or manual resume
			for ( let i = 0; i < this.samplesCount; i ++ ) {

				const s = this.samples[ i ];
				const dSq = ( s.point.x - _carPos.x ) ** 2 + ( s.point.z - _carPos.z ) ** 2;
				if ( dSq < minDistSq ) {

					minDistSq = dSq;
					bestIdx = i;

				}

			}
			this.needsGlobalSplineLocate = false;

		} else {

			// Continuous forward progression: tight backward window (-5), forward tracking (+25)
			for ( let i = - 5; i <= 25; i ++ ) {

				const idx = ( this.closestSampleIdx + i + this.samplesCount ) % this.samplesCount;
				const s = this.samples[ idx ];
				const dSq = ( s.point.x - _carPos.x ) ** 2 + ( s.point.z - _carPos.z ) ** 2;

				if ( dSq < minDistSq ) {

					minDistSq = dSq;
					bestIdx = idx;

				}

			}

		}

		this.closestSampleIdx = bestIdx;
		const closestSample = this.samples[ bestIdx ];

		// Signed lateral offset from centerline (+ right, - left)
		_toTarget.subVectors( _carPos, closestSample.point );
		const trackOffset = _toTarget.dot( _right );
		this.sensors.trackOffset = trackOffset;

		// Calculate track curvature ahead of vehicle
		const aheadSampleIdx = ( bestIdx + 15 ) % this.samplesCount;
		const curvature = Math.abs( closestSample.tangent.angleTo( this.samples[ aheadSampleIdx ].tangent ) );
		this.sensors.curvature = curvature;

		// Adaptive lookahead along spline: shorter in tight corners to hug apex, longer on straights
		const baseLookahead = curvature > 0.25 ? 2.4 : THREE.MathUtils.lerp( 2.8, 3.8, trueSpeed / 1.15 );
		const lookaheadDist = THREE.MathUtils.clamp( baseLookahead, 2.0, 4.2 );
		this.telemetry.lookaheadDist = lookaheadDist;

		// Find target point lookaheadDist meters ahead on the spline
		let targetDistance = ( closestSample.dist + lookaheadDist ) % this.totalSplineLength;
		if ( targetDistance < 0 ) targetDistance += this.totalSplineLength;
		let targetU = targetDistance / this.totalSplineLength;
		targetU = THREE.MathUtils.clamp( targetU, 0.0, 0.9999 );
		const targetPoint = this.spline.getPointAt( targetU );
		const targetTangent = this.spline.getTangentAt( targetU ).normalize();

		if ( this.targetRing && this.targetRing.visible ) {

			this.targetRing.position.set( targetPoint.x, 0.08, targetPoint.z );

		}

		// ─── 4. Pure Pursuit & Stanley Steering Controller ──────────────────────────
		let steer = 0;
		let throttle = 1.0;

		if ( this.config.useCustomCode && this.customLogicFunction ) {

			try {

				const res = this.customLogicFunction( this.sensors, {
					trueSpeed,
					lookaheadDist,
					targetPoint,
				} );

				if ( res && typeof res.steer === 'number' ) steer = res.steer;
				if ( res && typeof res.throttle === 'number' ) throttle = res.throttle;

			} catch ( err ) {

				console.warn( 'Custom logic error:', err );
				this.config.useCustomCode = false;

			}

		}

		if ( ! this.config.useCustomCode ) {

			// A. Pure Pursuit Steering (NPC Race Car Math)
			// In Vehicle.js:
			//   targetAngular = - controlsInput.x * steerSensitivity
			//   rotateY(positive) turns towards +X (RIGHT).
			//   rotateY(negative) turns towards -X (LEFT).
			// Therefore:
			//   To turn RIGHT: controlsInput.x MUST BE NEGATIVE (-1.0).
			//   To turn LEFT:  controlsInput.x MUST BE POSITIVE (+1.0).
			_toTarget.subVectors( targetPoint, _carPos );
			_toTarget.y = 0;

			const distToTarget = Math.max( _toTarget.length(), 0.5 );
			const dLateral = _toTarget.dot( _right );    // >0 when target is to the RIGHT
			const dForward = _toTarget.dot( _forward );

			// Angle to target in vehicle local frame: >0 when target is to the RIGHT
			const angleToTarget = Math.atan2( dLateral, Math.max( dForward, 0.2 ) );

			// Heading alignment to spline tangent
			const headingCross = _forward.x * targetTangent.z - _forward.z * targetTangent.x;

			// Cross-track error correction: trackOffset > 0 means car is right of centerline
			const crossTrackCorrection = ( trackOffset / 1.5 );

			// Compute base racing line steering:
			// If target is to RIGHT (angleToTarget > 0), rawSteer must be NEGATIVE to turn RIGHT!
			// If target is to LEFT (angleToTarget < 0), rawSteer must be POSITIVE to turn LEFT!
			let rawSteer = - ( angleToTarget * 2.2 + headingCross * 0.8 - crossTrackCorrection * 0.35 ) * ( this.config.steerGain / 3.0 );

			// B. Real-Time Obstacle Avoidance Corridor & Dynamic Evasion
			let obstacleSteerOffset = 0;

			// Check for obstacle threats in forward corridor:
			if ( frontThreat < 15.0 || timeToCollision < 2.0 ) {

				const threatFactor = THREE.MathUtils.clamp( ( 15.0 - frontThreat ) / 15.0, 0.0, 1.0 );

				// Determine evasion corridor:
				let evadeDir = 0;
				if ( this.sensors.fwdLeft < this.sensors.fwdRight - 0.75 ) {

					// Obstacle is on LEFT -> Evade to RIGHT (negative steer)
					evadeDir = - 1.0;

				} else if ( this.sensors.fwdRight < this.sensors.fwdLeft - 0.75 ) {

					// Obstacle is on RIGHT -> Evade to LEFT (positive steer)
					evadeDir = 1.0;

				} else {

					// Obstacle dead ahead -> choose side with greatest clearance
					evadeDir = leftClearance >= rightClearance ? 1.0 : - 1.0;

				}

				// Boundary clearance check: prevent evading into a side wall
				if ( evadeDir > 0 && this.sensors.sideLeft < 1.8 ) {

					evadeDir = - 1.0; // Left wall too close, steer right

				} else if ( evadeDir < 0 && this.sensors.sideRight < 1.8 ) {

					evadeDir = 1.0; // Right wall too close, steer left

				}

				const avoidStrength = threatFactor * ( 1.2 + threatFactor * 1.5 ) * this.config.wallAvoidance;
				obstacleSteerOffset = evadeDir * avoidStrength;

			}

			// B2. Dedicated Interactive Mouse Hazard Evader & Dynamic Evasion
			let mouseHazardThreat = false;
			let isMouseEmergency = false;
			if ( this.mouseObstacleActive ) {

				_toTarget.subVectors( this.mouseObstaclePos, _carPos );
				_toTarget.y = 0;
				const distToMouse = _toTarget.length();
				this.telemetry.mouseObstacleDist = distToMouse;

				const fwdDist = _toTarget.dot( _forward );
				const latDist = _toTarget.dot( _right ); // >0 = obstacle is to RIGHT, <0 = obstacle is to LEFT

				const hazardCorridor = this.mouseObstacleRadius + 1.5;
				if ( fwdDist > 0 && fwdDist < 18.0 && Math.abs( latDist ) < hazardCorridor ) {

					mouseHazardThreat = true;
					const urgency = THREE.MathUtils.clamp( ( 18.0 - fwdDist ) / 18.0, 0.0, 1.0 );

					// If obstacle is to RIGHT (latDist >= 0), steer LEFT (+1.0)
					// If obstacle is to LEFT (latDist < 0), steer RIGHT (-1.0)
					let mouseEvadeDir = latDist >= 0 ? 1.0 : - 1.0;

					// Respect wall margins so we don't steer into an adjacent barrier
					if ( mouseEvadeDir > 0 && this.sensors.sideLeft < 1.6 ) {

						mouseEvadeDir = - 1.0;

					} else if ( mouseEvadeDir < 0 && this.sensors.sideRight < 1.6 ) {

						mouseEvadeDir = 1.0;

					}

					const mouseEvadeStrength = urgency * ( 1.8 + urgency * 1.8 ) * this.config.wallAvoidance;
					obstacleSteerOffset += mouseEvadeDir * mouseEvadeStrength;

					if ( fwdDist < 4.5 ) {

						isMouseEmergency = true;

					}

				}

			}

			// C. High-Priority Boundary Wall Repulsion & Glancing Deflection
			const margin = this.config.wallSafetyMargin;
			const avoidGain = this.config.wallAvoidance;
			let boundaryPush = 0;

			// Left wall close -> turn RIGHT (negative steer)
			if ( this.sensors.sideLeft < margin ) {

				const push = ( ( margin - this.sensors.sideLeft ) / margin ) * avoidGain * 1.4;
				boundaryPush -= push;

			}

			// Right wall close -> turn LEFT (positive steer)
			if ( this.sensors.sideRight < margin ) {

				const push = ( ( margin - this.sensors.sideRight ) / margin ) * avoidGain * 1.4;
				boundaryPush += push;

			}

			// Forward-diagonal whiskers (early apex turn-in protection)
			if ( this.sensors.diagLeft < margin * 1.15 ) {

				const push = ( ( margin * 1.15 - this.sensors.diagLeft ) / ( margin * 1.15 ) ) * avoidGain * 0.95;
				boundaryPush -= push;

			}

			if ( this.sensors.diagRight < margin * 1.15 ) {

				const push = ( ( margin * 1.15 - this.sensors.diagRight ) / ( margin * 1.15 ) ) * avoidGain * 0.95;
				boundaryPush += push;

			}

			// Combine all steering components
			rawSteer += obstacleSteerOffset + boundaryPush;

			// Snappy, high-bandwidth steering filter with rapid counter-steering
			const filterRate = Math.sign( this.smoothedSteer ) !== Math.sign( rawSteer ) && Math.abs( rawSteer ) > 0.1 ? 26 : 18;
			this.smoothedSteer = THREE.MathUtils.lerp( this.smoothedSteer, rawSteer, Math.min( dt * filterRate, 1.0 ) );
			steer = this.smoothedSteer;

			// D. Autonomous Decision-Making Speed Governor & Emergency Braking
			let targetSpeed = this.config.targetSpeed || 1.15;
			let decisionStatus = 'RACING LINE';

			// 0. Mouse Hazard Speed & Decision Governor
			if ( this.mouseObstacleActive && mouseHazardThreat ) {

				const fwdDist = _toTarget.dot( _forward );
				if ( isMouseEmergency ) {

					targetSpeed = 0.05;
					decisionStatus = 'EMERGENCY BRAKE (MOUSE HAZARD)';

				} else {

					targetSpeed = Math.min( targetSpeed, THREE.MathUtils.clamp( ( fwdDist - 2.0 ) * 0.12, 0.30, 0.85 ) );
					decisionStatus = 'AVOIDING MOUSE HAZARD';

				}

			}

			// 1. Curvature speed reduction (Trail brake into turns)
			if ( curvature > 0.18 ) {

				const cornerSpeed = THREE.MathUtils.clamp( 1.15 - curvature * 1.4 * this.config.cornerBraking, 0.55, 0.95 );
				targetSpeed = Math.min( targetSpeed, cornerSpeed );
				if ( ! mouseHazardThreat ) decisionStatus = 'CORNER APEX';

			}

			// 2. Obstacle approach speed regulation (v_safe = sqrt(2 * a * d))
			if ( frontThreat < 13.0 ) {

				const safeDist = Math.max( 0.0, frontThreat - 1.8 );
				const obstacleSafeSpeed = THREE.MathUtils.clamp( Math.sqrt( safeDist * 0.22 ), 0.35, 1.15 );
				targetSpeed = Math.min( targetSpeed, obstacleSafeSpeed );
				if ( ! mouseHazardThreat ) decisionStatus = 'AVOIDING OBSTACLE';

			}

			// 3. Emergency Braking Decision: Obstacle imminent (< 4.2m, closing rapidly, high speed)
			const isEmergencyThreat = isMouseEmergency || ( frontThreat < 4.2 && timeToCollision < 0.75 && trueSpeed > 2.5 );
			if ( isEmergencyThreat ) {

				targetSpeed = 0.05;
				if ( ! isMouseEmergency ) decisionStatus = 'EMERGENCY BRAKE';

			}

			// 4. Proactive Throttle & Braking Output
			const curLinear = vehicle.linearSpeed;
			if ( isEmergencyThreat ) {

				// Maximum threshold braking
				throttle = - 0.85;

			} else if ( curLinear < targetSpeed - 0.04 ) {

				// Acceleration zone
				throttle = this.config.maxThrottle;
				if ( decisionStatus === 'RACING LINE' ) decisionStatus = 'FULL THROTTLE';

			} else if ( curLinear > targetSpeed + 0.08 ) {

				// Progressive braking zone
				const overspeed = curLinear - targetSpeed;
				throttle = - THREE.MathUtils.clamp( overspeed * 2.5, 0.25, 0.75 );
				if ( decisionStatus === 'RACING LINE' ) decisionStatus = 'TRAIL BRAKING';

			} else {

				// Cruising zone
				throttle = THREE.MathUtils.clamp( ( targetSpeed / 1.15 ) * 0.85, 0.35, 0.90 );
				if ( decisionStatus === 'RACING LINE' ) decisionStatus = 'CRUISE CONTROL';

			}

			this.telemetry.status = decisionStatus;
			this.decisionState = decisionStatus;

		}

		// Prevent unintentional reverse gear when near standstill in forward mode
		if ( this.recoveryPhase === 0 && vehicle.linearSpeed <= 0.05 && throttle < 0 ) {

			throttle = 0.05;

		}

		steer = THREE.MathUtils.clamp( steer, - 1.0, 1.0 );
		throttle = THREE.MathUtils.clamp( throttle, - 1.0, 1.0 );

		this.updateTelemetryHUD( throttle, steer );

		return { x: steer, z: throttle, touchActive: false };

	}

	updateTelemetryHUD( throttle, steer ) {

		this.telemetry.steer = steer;
		this.telemetry.throttle = throttle;

		if ( ! this.tunerPanel || ! this.tunerPanel.classList.contains( 'open' ) ) return;

		const speedKmh = ( this.telemetry.trueSpeed * 18.0 ).toFixed( 1 );
		const mStatus = this.tunerPanel.querySelector( '#ai-m-status' );
		if ( mStatus ) {

			mStatus.textContent = this.telemetry.status;
			mStatus.style.color = this.enabled ? ( this.telemetry.status.includes( 'BRAKING' ) ? '#f59e0b' : '#22c55e' ) : '#64748b';

		}

		const setVal = ( id, text ) => {

			const el = this.tunerPanel.querySelector( id );
			if ( el ) el.textContent = text;

		};

		setVal( '#ai-m-speed', `${ speedKmh } km/h` );
		setVal( '#ai-m-steer', steer.toFixed( 2 ) );
		setVal( '#ai-m-throttle', `${ Math.round( throttle * 100 ) }%` );
		setVal( '#ai-m-offset', `${ this.sensors.trackOffset > 0 ? '+' : '' }${ this.sensors.trackOffset.toFixed( 2 ) }m` );
		setVal( '#ai-m-nearest', `${ this.sensors.nearestWallDist.toFixed( 1 ) }m` );
		setVal( '#ai-m-rfwd', `${ this.sensors.fwdFar.toFixed( 1 ) }m` );
		setVal( '#ai-m-rfwdlr', `${ this.sensors.fwdLeft.toFixed( 1 ) }m / ${ this.sensors.fwdRight.toFixed( 1 ) }m` );
		setVal( '#ai-m-rdiaglr', `${ this.sensors.diagLeft.toFixed( 1 ) }m / ${ this.sensors.diagRight.toFixed( 1 ) }m` );
		setVal( '#ai-m-rsidelr', `${ this.sensors.sideLeft.toFixed( 1 ) }m / ${ this.sensors.sideRight.toFixed( 1 ) }m` );

		const mouseHazardVal = this.tunerPanel.querySelector( '#ai-m-mouse-hazard' );
		if ( mouseHazardVal ) {

			if ( this.mouseObstacleActive ) {

				const dist = Number.isFinite( this.telemetry.mouseObstacleDist ) ? `${ this.telemetry.mouseObstacleDist.toFixed( 1 ) }m` : '--';
				mouseHazardVal.textContent = `ACTIVE (${ dist } dist)`;
				mouseHazardVal.style.color = '#ef4444';

			} else {

				mouseHazardVal.textContent = 'DISABLED [Press M]';
				mouseHazardVal.style.color = '#64748b';

			}

		}

	}

	realignToTrackSpline( vehicle ) {

		if ( ! vehicle || ! this.spline || this.samples.length === 0 ) return;

		let bestDistSq = Infinity;
		let bestSample = this.samples[ 0 ];
		const targetPos = vehicle.spherePos;

		for ( let i = 0; i < this.samplesCount; i ++ ) {

			const s = this.samples[ i ];
			const dSq = ( s.point.x - targetPos.x ) ** 2 + ( s.point.z - targetPos.z ) ** 2;
			if ( dSq < bestDistSq ) {

				bestDistSq = dSq;
				bestSample = s;

			}

		}

		if ( bestSample ) {

			const tangentAngle = Math.atan2( bestSample.tangent.x, bestSample.tangent.z );
			if ( typeof vehicle.repositionTo === 'function' ) {

				vehicle.repositionTo( bestSample.point.x, 0.5, bestSample.point.z, tangentAngle );

			}

		}

	}

}
