import { useEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { ARM_JOINT_NAMES, type ManipulatorState } from '../hooks/useManipulatorState'
import './ManipulatorViewer.css'

type JointSpec = {
  name: (typeof ARM_JOINT_NAMES)[number]
  axis: 'x' | 'y' | 'z'
  origin: [number, number, number]
  limit: [number, number]
  mesh?: string
  visualOrigin?: [number, number, number]
  visualRpy?: [number, number, number]
}

const JOINT_SPECS: JointSpec[] = [
  {
    name: 'arm_j1',
    axis: 'z',
    origin: [0, 0, 0],
    limit: [-3.1415926536, 3.1415926536],
    mesh: 'rover_manipulator_link1.stl',
    visualOrigin: [0, 0, 0.032],
    visualRpy: [0, 0, Math.PI],
  },
  {
    name: 'arm_j2',
    axis: 'y',
    origin: [0.07, 0, 0.136],
    limit: [-0.7853981634, 1.5707963268],
    mesh: 'rover_manipulator_link2.stl',
    visualOrigin: [-0.07, 0, -0.104],
    visualRpy: [0, 0, Math.PI],
  },
  {
    name: 'arm_j3',
    axis: 'y',
    origin: [0, 0, 0.495],
    limit: [-1.745329252, 0],
    mesh: 'rover_manipulator_link3.stl',
    visualOrigin: [-0.07, 0.0001, -0.599],
    visualRpy: [0, 0, Math.PI],
  },
  {
    name: 'arm_j4',
    axis: 'z',
    origin: [0.07, 0, -0.138],
    limit: [-3.490658504, 3.490658504],
    mesh: 'rover_manipulator_link4.stl',
    visualOrigin: [-0.14, 0, -0.461],
    visualRpy: [0, 0, Math.PI],
  },
  {
    name: 'arm_j5',
    axis: 'y',
    origin: [0, 0, -0.26],
    limit: [0, 3.490658504],
    mesh: 'rover_manipulator_link5.stl',
    visualOrigin: [-0.14, 0, -0.201],
    visualRpy: [0, 0, Math.PI],
  },
  {
    name: 'arm_j6',
    axis: 'x',
    origin: [0.116, 0, 0],
    limit: [-3.1415926536, 3.1415926536],
  },
]

const MODEL_BASE_PATH = '/models/manipulator'
const VIEW_TARGET = new THREE.Vector3(0.08, 0, 0.25)
const EEF_OFFSET: [number, number, number] = [0.215, 0, 0]
const LIMIT_MARGIN_RAD = 0.15
const ARM_COLOR = 0x9aa7b8
const LIMIT_COLOR = 0xff4d4d
const FRONT_COLOR = 0x35d3c3

const makeArmMaterial = () =>
  new THREE.MeshStandardMaterial({
    color: ARM_COLOR,
    metalness: 0.18,
    roughness: 0.62,
  })

const applyAxisRotation = (joint: THREE.Object3D, axis: JointSpec['axis'], value: number) => {
  joint.rotation.set(0, 0, 0)
  joint.rotation[axis] = value
}

const isNearLimit = (value: number, [lower, upper]: [number, number]) =>
  value - lower <= LIMIT_MARGIN_RAD || upper - value <= LIMIT_MARGIN_RAD

type ManipulatorViewerProps = {
  embedded?: boolean
  state: ManipulatorState
}

const ManipulatorViewer = ({ embedded = false, state }: ManipulatorViewerProps) => {
  const mountRef = useRef<HTMLDivElement | null>(null)
  const jointRefs = useRef<Record<string, THREE.Object3D>>({})
  const materialRefs = useRef<Record<string, THREE.MeshStandardMaterial>>({})

  const statusText = useMemo(() => {
    if (!state.connected) return 'rosbridge offline'
    if (state.lastUpdateMs == null) return 'waiting for /joint_states'
    if (state.stale) return 'joint states stale'
    if (state.missingJoints.length > 0) return `missing ${state.missingJoints.join(', ')}`
    return 'live joint states'
  }, [state.connected, state.lastUpdateMs, state.missingJoints, state.stale])

  useEffect(() => {
    const mount = mountRef.current
    if (!mount) return

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x0b1220)

    const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 20)
    camera.position.set(1.4, -1.9, 1.1)
    camera.lookAt(VIEW_TARGET)

    const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    mount.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.target.copy(VIEW_TARGET)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.enablePan = false
    controls.minDistance = 0.7
    controls.maxDistance = 4
    controls.update()

    scene.add(new THREE.HemisphereLight(0xdde8ff, 0x172033, 1.4))
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2)
    keyLight.position.set(2.5, -3, 4)
    scene.add(keyLight)

    const root = new THREE.Group()
    root.rotation.x = -Math.PI / 2
    scene.add(root)

    const grid = new THREE.GridHelper(1.8, 12, 0x355066, 0x203040)
    grid.rotation.x = Math.PI / 2
    root.add(grid)

    const frontArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0.02),
      0.72,
      FRONT_COLOR,
      0.14,
      0.08,
    )
    root.add(frontArrow)

    const loader = new STLLoader()
    let parent: THREE.Object3D = root
    const jointMap: Record<string, THREE.Object3D> = {}
    const materialMap: Record<string, THREE.MeshStandardMaterial> = {}

    for (const spec of JOINT_SPECS) {
      const joint = new THREE.Object3D()
      joint.position.set(...spec.origin)
      parent.add(joint)
      jointMap[spec.name] = joint
      const material = makeArmMaterial()
      materialMap[spec.name] = material

      if (spec.mesh) {
        loader.load(
          `${MODEL_BASE_PATH}/${spec.mesh}`,
          (geometry) => {
            geometry.computeVertexNormals()
            const mesh = new THREE.Mesh(geometry, material)
            mesh.scale.setScalar(0.001)
            mesh.position.set(...(spec.visualOrigin ?? [0, 0, 0]))
            mesh.rotation.set(...(spec.visualRpy ?? [0, 0, 0]))
            joint.add(mesh)
          },
          undefined,
          () => {
            const fallback = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.24), material)
            fallback.position.set(0, 0, 0.12)
            joint.add(fallback)
          },
        )
      } else {
        const wrist = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 0.1), material)
        wrist.position.set(0.05, 0, 0)
        joint.add(wrist)
      }

      parent = joint
    }

    const eefFrame = new THREE.Object3D()
    eefFrame.position.set(...EEF_OFFSET)
    parent.add(eefFrame)

    const eefAxes = new THREE.AxesHelper(0.18)
    eefFrame.add(eefAxes)

    const resize = () => {
      const rect = mount.getBoundingClientRect()
      const width = Math.max(1, rect.width)
      const height = Math.max(1, rect.height)
      renderer.setSize(width, height, false)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
    }

    const observer = new ResizeObserver(resize)
    observer.observe(mount)
    resize()

    let raf = 0
    const render = () => {
      controls.update()
      renderer.render(scene, camera)
      raf = requestAnimationFrame(render)
    }
    render()

    jointRefs.current = jointMap
    materialRefs.current = materialMap

    return () => {
      cancelAnimationFrame(raf)
      observer.disconnect()
      jointRefs.current = {}
      materialRefs.current = {}
      controls.dispose()
      mount.removeChild(renderer.domElement)
      renderer.dispose()
      Object.values(materialMap).forEach((material) => material.dispose())
    }
  }, [])

  useEffect(() => {
    for (const spec of JOINT_SPECS) {
      const joint = jointRefs.current[spec.name]
      if (joint) applyAxisRotation(joint, spec.axis, state.joints[spec.name])
      const material = materialRefs.current[spec.name]
      if (material) {
        material.color.setHex(isNearLimit(state.joints[spec.name], spec.limit) ? LIMIT_COLOR : ARM_COLOR)
      }
    }
  }, [state.joints])

  return (
    <article className={`manipulator-viewer ${embedded ? 'manipulator-viewer--embedded' : 'card'}`}>
      <header className="manipulator-viewer__header">
        <h3>Manipulator</h3>
        <span className={`pill ${state.connected && !state.stale ? 'pill--on' : 'pill--off'}`}>
          {statusText}
        </span>
      </header>
      <div className="manipulator-viewer__canvas" ref={mountRef} />
      <p className="manipulator-viewer__hint">
        {state.ageMs == null ? 'No joint state received' : `Last update ${Math.round(state.ageMs)} ms ago`}
      </p>
    </article>
  )
}

export default ManipulatorViewer
