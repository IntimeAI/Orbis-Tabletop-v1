"""Franka robot pick-and-place task with office table scene."""

from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import XFormPrim
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
import logging
import math
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdLux, Sdf, Gf
from typing import Optional, Dict, Any
from omni.kit.viewport.utility import get_active_viewport

logger = logging.getLogger(__name__)


class FrankaPlaying(BaseTask):
    """Franka robot pick-and-place task."""

    DEFAULT_GOAL_POSITION = np.array([0.615, 0.15, 0.09])
    DEFAULT_Z_OFFSET = 0.05
    TASK_COMPLETION_THRESHOLD = 0.02

    DEFAULT_USD_PATH = (
        "./Orbis-Tabletop-v1/Tabletop/OfficeTables/office_table00001/office_table00001.usdc"
    )
    TABLE_PRIM_PATH = "/World/OfficeTable"
    FRANKA_PRIM_PATH = "/World/Fancy_Franka"
    FRANKA_NAME = "fancy_franka"

    CAMERA_PRIM_PATH = "/World/Camera"
    CAMERA_POSITION = Gf.Vec3d(0.615, 0.0, 3.5)
    CAMERA_TARGET = Gf.Vec3d(0.615, 0.0, 0.0)

    TABLE_TRANSLATION = (0.8, 0.0, -1.2)
    TABLE_ROTATION_Z = -90.0
    TABLE_SCALE = (1.0, 1.0, 1.0)

    ENV_PRIM_PATH = "/Environment"
    DOME_LIGHT_INTENSITY = 1000
    DISTANT_LIGHT_INTENSITY = 1000
    DISTANT_LIGHT_ROTATION = (45.0, 35.0, 0.0)
    DEFAULT_LIGHT_COLOR = Gf.Vec3f(1.0, 1.0, 1.0)

    def __init__(
        self,
        name: str,
        usd_path: str = DEFAULT_USD_PATH,
        goal_position: Optional[np.ndarray] = None,
        z_offset: float = DEFAULT_Z_OFFSET,
    ):
        """Initialize task with scene parameters."""
        super().__init__(name=name, offset=None)

        self._z_offset = z_offset
        self._goal_position = (
            goal_position
            if goal_position is not None
            else self.DEFAULT_GOAL_POSITION.copy()
        )
        self._usd_path = usd_path

        self._task_achieved = False

        self._target_object: Optional[XFormPrim] = None
        self._target_prim_path: Optional[str] = None
        self._franka: Optional[Franka] = None

        logger.info(f"Initialized task: {name}, goal position: {self._goal_position}")

    def set_up_scene(self, scene) -> None:
        """Set up scene with table, robot, and target object."""
        super().set_up_scene(scene)

        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("Failed to get USD stage")

            self._load_office_table(stage)

            self._franka = scene.add(
                Franka(
                    prim_path=self.FRANKA_PRIM_PATH,
                    name=self.FRANKA_NAME,
                )
            )
            logger.info(
                f"Successfully added Franka robot to scene: {self.FRANKA_PRIM_PATH}"
            )

            self._configure_table_transform(stage)

            self._find_target_object(stage)

            if self._target_object is None:
                logger.warning("Target object not found, task may not complete")

            self._setup_light(stage)

            self._setup_camera(stage)

        except Exception as e:
            logger.error(f"Scene setup failed: {e}", exc_info=True)
            raise

    def _load_office_table(self, stage) -> None:
        """Load office table USD asset."""
        table_prim_path = Sdf.Path(self.TABLE_PRIM_PATH)

        if not stage.GetPrimAtPath(table_prim_path):
            try:
                table_prim = stage.DefinePrim(table_prim_path, "Xform")
                table_prim.GetReferences().AddReference(self._usd_path)
                logger.info(
                    f"Successfully loaded USD asset to {table_prim_path}, path: {self._usd_path}"
                )
            except Exception as e:
                logger.error(f"Failed to load USD asset: {e}")
                raise
        else:
            logger.info(f"Asset {table_prim_path} already exists, skipping load")

    def _configure_table_transform(self, stage) -> None:
        """Configure table transform."""
        office_table_prim = stage.GetPrimAtPath(self.TABLE_PRIM_PATH)

        if not office_table_prim or not office_table_prim.IsValid():
            logger.warning(f"Cannot find office table prim: {self.TABLE_PRIM_PATH}")
            return

        try:
            xformable = UsdGeom.Xformable(office_table_prim)
            xformable.ClearXformOpOrder()

            translate_op = xformable.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            translate_op.Set(self.TABLE_TRANSLATION)

            rotate_op = xformable.AddRotateZOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            rotate_op.Set(self.TABLE_ROTATION_Z)

            scale_op = xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
            scale_op.Set(self.TABLE_SCALE)

            logger.info(
                f"Successfully configured table transform: translation={self.TABLE_TRANSLATION}, rotationZ={self.TABLE_ROTATION_Z}"
            )

        except Exception as e:
            logger.error(f"Failed to configure table transform: {e}")

    def _find_target_object(self, stage) -> None:
        """Find target object (objects starting with "Chicken")."""
        for prim in stage.Traverse():
            prim_name = prim.GetName()
            if prim_name.startswith("Chicken"):
                prim_path = str(prim.GetPath())
                self._target_prim_path = prim_path

                try:
                    self._target_object = XFormPrim(
                        prim_paths_expr=prim_path, name="target_object"
                    )
                    logger.info(f"Found target object: {prim_path}")
                except Exception as e:
                    logger.error(f"Failed to create target object XFormPrim: {e}")
                    self._target_object = None
                break

    def _setup_light(self, stage) -> None:
        """Setup scene lighting with dome and distant lights."""
        try:
            env_path = Sdf.Path(self.ENV_PRIM_PATH)
            env_prim = stage.GetPrimAtPath(env_path)

            if not env_prim or not env_prim.IsValid():
                logger.debug(f"Creating environment prim at {self.ENV_PRIM_PATH}")
                env_prim = stage.DefinePrim(env_path, "Xform")

            dome_path = env_path.AppendChild("DefaultDomeLight")
            if not stage.GetPrimAtPath(dome_path):
                dome = UsdLux.DomeLight.Define(stage, dome_path)
                dome.CreateIntensityAttr().Set(self.DOME_LIGHT_INTENSITY)
                dome.CreateColorAttr().Set(self.DEFAULT_LIGHT_COLOR)
                logger.debug(
                    f"Created dome light with intensity {self.DOME_LIGHT_INTENSITY}"
                )
            else:
                logger.debug(f"Dome light already exists at {dome_path}")

            distant_path = env_path.AppendChild("DefaultDistantLight")
            if not stage.GetPrimAtPath(distant_path):
                distant = UsdLux.DistantLight.Define(stage, distant_path)
                distant.CreateIntensityAttr().Set(self.DISTANT_LIGHT_INTENSITY)
                distant.CreateColorAttr().Set(self.DEFAULT_LIGHT_COLOR)

                xformable = UsdGeom.Xformable(distant.GetPrim())
                rotate_op = xformable.AddRotateXYZOp(
                    precision=UsdGeom.XformOp.PrecisionDouble
                )
                rotate_op.Set(self.DISTANT_LIGHT_ROTATION)

                logger.debug(
                    f"Created distant light with intensity {self.DISTANT_LIGHT_INTENSITY} and rotation (45, -30, 0)"
                )
            else:
                logger.debug(f"Distant light already exists at {distant_path}")

            logger.info("Scene lighting setup completed successfully")

        except Exception as e:
            logger.error(f"Failed to setup scene lighting: {e}")
            raise RuntimeError(f"Lighting setup failed: {e}") from e

    def _setup_camera(self, stage) -> None:
        """Setup camera for rendering."""
        camera_prim_path = Sdf.Path(self.CAMERA_PRIM_PATH)

        if stage.GetPrimAtPath(camera_prim_path):
            logger.info(f"Camera {camera_prim_path} already exists, skipping creation")
            return

        try:
            camera = UsdGeom.Camera.Define(stage, camera_prim_path)

            camera.CreateFocalLengthAttr(24.0)
            camera.CreateFocusDistanceAttr(400.0)

            camera_prim = stage.GetPrimAtPath(camera_prim_path)
            xformable = UsdGeom.Xformable(camera_prim)

            translate_op = xformable.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            )
            translate_op.Set(self.CAMERA_POSITION)

            # Calculate look-at rotation (USD cameras look down -Z axis)
            forward = (self.CAMERA_TARGET - self.CAMERA_POSITION).GetNormalized()

            world_up = Gf.Vec3d(0, 0, 1)

            right = Gf.Cross(forward, world_up)
            if right.GetLength() < 0.001:
                world_up = Gf.Vec3d(0, 1, 0)
                right = Gf.Cross(forward, world_up)
            right = right.GetNormalized()

            up = Gf.Cross(right, forward).GetNormalized()

            rotation_matrix = Gf.Matrix3d(
                right[0],
                up[0],
                -forward[0],
                right[1],
                up[1],
                -forward[1],
                right[2],
                up[2],
                -forward[2],
            )

            # Extract euler angles from rotation matrix
            sy = math.sqrt(
                rotation_matrix[0][0] * rotation_matrix[0][0]
                + rotation_matrix[1][0] * rotation_matrix[1][0]
            )

            singular = sy < 1e-6

            if not singular:
                roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
                pitch = math.atan2(-rotation_matrix[2][0], sy)
                yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
            else:
                roll = math.atan2(-rotation_matrix[1][2], rotation_matrix[1][1])
                pitch = math.atan2(-rotation_matrix[2][0], sy)
                yaw = 0

            rotate_xyz_op = xformable.AddRotateXYZOp(
                precision=UsdGeom.XformOp.PrecisionFloat
            )
            rotate_xyz_op.Set(
                Gf.Vec3f(
                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw) - 90
                )
            )
            logger.info(
                f"Successfully created camera at {camera_prim_path}, "
                f"position: {self.CAMERA_POSITION}, target: {self.CAMERA_TARGET}"
            )

        except Exception as e:
            logger.error(f"Failed to create camera: {e}")

    def get_observations(self) -> Dict[str, Any]:
        """Get current observations."""
        if self._franka is None:
            logger.warning("Franka robot not initialized")
            return {}

        try:
            current_joint_positions = self._franka.get_joint_positions()
            observations = {
                self._franka.name: {
                    "joint_positions": current_joint_positions,
                },
            }

            if self._target_object is not None:
                target_positions, _ = self._target_object.get_world_poses()
                if target_positions is not None and len(target_positions) > 0:
                    observations["target_object"] = {
                        "position": target_positions[0]
                        + np.array([0, 0, self._z_offset]),
                        "goal_position": self._goal_position,
                    }

            return observations

        except Exception as e:
            logger.error(f"Failed to get observations: {e}")
            return {}

    def pre_step(self, control_index: int, simulation_time: float) -> None:
        """Check if task is complete."""
        if self._task_achieved or self._target_object is None:
            return

        try:
            target_positions, _ = self._target_object.get_world_poses()
            if target_positions is None or len(target_positions) == 0:
                return

            current_position = target_positions[0] + np.array([0, 0, self._z_offset])
            distance = np.linalg.norm(self._goal_position - current_position)

            if distance < self.TASK_COMPLETION_THRESHOLD:
                self._task_achieved = True
                logger.info(f"Task completed! Distance: {distance:.4f}m")

        except Exception as e:
            logger.error(f"pre_step execution failed: {e}")

    def post_reset(self) -> None:
        """Reset task state."""
        try:
            if self._franka is not None and self._franka.gripper is not None:
                self._franka.gripper.set_joint_positions(
                    self._franka.gripper.joint_opened_positions
                )
            self._task_achieved = False
            logger.info("Task reset")

        except Exception as e:
            logger.error(f"Task reset failed: {e}")

    @property
    def is_task_achieved(self) -> bool:
        """Return whether the task is completed."""
        return self._task_achieved


class TablePickup(BaseSample):
    """Table pickup example."""

    def __init__(self) -> None:
        """Initialize example."""
        super().__init__()
        self._world = None
        self._franka: Optional[Franka] = None
        self._controller: Optional[PickPlaceController] = None
        self._task: Optional[FrankaPlaying] = None
        self._movie_capture = None
        self._recording_active = False

    def setup_scene(self) -> None:
        """Set up scene and add task."""
        world = self.get_world()
        if world is None:
            raise RuntimeError("Failed to get world object")

        self._task = FrankaPlaying(name="table_pickup")
        world.add_task(self._task)
        logger.info("Scene setup complete")

    async def setup_post_load(self) -> None:
        """Initialize controller and physics callback."""
        try:
            self._world = self.get_world()
            if self._world is None:
                raise RuntimeError("Failed to get world object")

            self._franka = self._world.scene.get_object(FrankaPlaying.FRANKA_NAME)
            if self._franka is None:
                raise RuntimeError(
                    f"Cannot find Franka robot: {FrankaPlaying.FRANKA_NAME}"
                )

            self._controller = PickPlaceController(
                name="pick_place_controller",
                gripper=self._franka.gripper,
                robot_articulation=self._franka,
            )

            self._setup_movie_capture()

            self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)

            await self._world.play_async()
            logger.info("Post-load setup complete")

        except Exception as e:
            logger.error(f"Post-load setup failed: {e}", exc_info=True)
            raise

    async def setup_post_reset(self) -> None:
        """Reset controller state."""
        try:
            if self._controller is not None:
                self._controller.reset()

            if self._world is not None:
                await self._world.play_async()

            logger.info("Post-reset setup complete")

        except Exception as e:
            logger.error(f"Post-reset setup failed: {e}")

    def _setup_movie_capture(self) -> None:
        """Setup movie capture for recording."""
        try:
            viewport_api = get_active_viewport()
            if viewport_api:
                viewport_api.set_active_camera(FrankaPlaying.CAMERA_PRIM_PATH)
                logger.info(f"Set active camera to: {FrankaPlaying.CAMERA_PRIM_PATH}")

            logger.info("Movie capture interface initialized")
        except Exception as e:
            logger.error(f"Failed to initialize movie capture: {e}")
            self._movie_capture = None

    def physics_step(self, step_size: float) -> None:
        """Execute pick and place actions."""
        if self._world is None or self._franka is None or self._controller is None:
            return

        try:
            current_observations = self._world.get_observations()

            if "target_object" not in current_observations:
                return

            if FrankaPlaying.FRANKA_NAME not in current_observations:
                logger.warning(
                    f"Missing robot observation data: {FrankaPlaying.FRANKA_NAME}"
                )
                return

            actions = self._controller.forward(
                picking_position=current_observations["target_object"]["position"],
                placing_position=current_observations["target_object"]["goal_position"],
                current_joint_positions=current_observations[FrankaPlaying.FRANKA_NAME][
                    "joint_positions"
                ],
            )

            self._franka.apply_action(actions)

            if self._controller.is_done():
                self._world.pause()
                logger.info("Controller finished, pausing simulation")

        except KeyError as e:
            logger.error(f"Observation data missing required key: {e}")
        except Exception as e:
            logger.error(f"Physics step execution failed: {e}")
