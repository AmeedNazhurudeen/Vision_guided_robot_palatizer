# Vision Guided Robot Palletizer

A Franka FR3 with a wrist-mounted RealSense D435 looks at a pile of bricks, finds the top layer, judges each brick's shade
(good or bad), picks them one by one, and stacks the good ones by colour (bad ones go to a bin). The same idea is simulated for a
whole brick factory line in Isaac Sim. Built at the Marmon x Franks hackathon (Team 8A).

Three deliverables, one folder each:

| Deliverable | Where |
|---|---|
| Real-brick classification (good or scrap, on real sample bricks) | `classification/real_bricks/` |
| Live robot demo (LEGO bricks stand in for fired bricks) | `robot/`, `perception/`, `calibration/`, `franka/` |
| Full-line factory simulation (kiln cart to pallet) | `simulation/` |

## Folder map

| Folder | What is in it |
|---|---|
| `robot/fr3_pose_pick/` | The ROS 2 package: **motion planning** (collision-checked straight lines, trajectory smoothing), pick and place, the pile sorter `sort_pile.py`, palletizing, recorded poses, all settings (`config/pose_pick.yaml`) and the unit tests |
| `franka/` | Franka's own software, as used on the robot: `franka_ros2` (drivers, MoveIt config, hand), `libfranka`, `franka_description` |
| `calibration/` | **Hand-eye calibration**: `results/` (the camera-to-hand result `charuco_result.json` and its samples) and `handeye_tool/` (the app that captures samples and solves it) |
| `perception/` | Camera and vision: D435 capture, SAM3 top-layer segmentation, grasp pose, colour classifier, Codex or Claude review, the colour-model setups. Run `run_pipeline_classifier.sh` here |
| `classification/real_bricks/` | The classifier for real (fired) bricks: SAM3 plus the frozen "Approach 17" model |
| `simulation/factory_line/` | The Isaac Sim ACME line: dehacking, reject robots, separator, gantries, pallets, with its validation and collision audit |
| `simulation/brick_scenes/` | Isaac Sim brick scenes (RGB-D like a real D435) and colour-model training from them |
| `docs/` | `sorter_documentation.md`: the full technical description of the sorter |

## Build and run the robot

```bash
cd ~/vision-guided-robot-palletizer
./build.sh                                   # once (compiles libfranka, so the first build takes a while)
source env.sh                                # in EVERY terminal: ROS 2 Humble, this workspace, ROS_DOMAIN_ID=77

# terminal 1: the robot, MoveIt and RViz
ros2 launch fr3_pose_pick pose_bringup.launch.py robot_ip:=192.168.3.10

# terminal 2: the sorter
ros2 run fr3_pose_pick sort_pile.py --config robot/fr3_pose_pick/config/pose_pick.yaml
```

A Franka workspace that is already built elsewhere can be reused: `export FRANKA_WS=<its folder>` before `./build.sh` and `source env.sh`.
Record the poses (`observe`, `bin`, `stack_<colour>`, `pile_corner_1/2`) with `record_pose.py` when the table layout changes:
see `robot/fr3_pose_pick/README.md` and `docs/sorter_documentation.md`.

The camera step needs a Python environment with SAM3 (conda env `sam3`, a CUDA GPU, Hugging Face access to `facebook/sam3`) and, for the
review step, the Codex CLI logged in (`--review claude` or `--review none` are the alternatives). See `perception/requirements.txt`.

**Safety:** the robot moves on its own. Keep the e-stop in reach. Only one robot stack may run in a ROS domain.

## Tests (no robot needed)

```bash
source env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q robot/fr3_pose_pick/test
```

## Simulation

```bash
~/env_isaacsim/bin/python simulation/factory_line/run_factory.py --camera Separator --start 105
```

Needs Isaac Sim (pip environment). Details, measured numbers and limits are in `simulation/factory_line/README.md`.
Generated output (`output/`) is not kept in the folder; rebuild it with `build_factory.py`.

## Team 8A

This was a team project. Roles are as the team listed them, and the code listed under a name is what that person's commits wrote.

| Who | Role | Code |
|---|---|---|
| Ameed Nazhurudeen | Robotics and motion planning | `robot/`: straight-line motion planning with collision and joint-swing checks, trajectory smoothing, grasp and place logic (pick centring, measured brick heights, recorded bin and stack poses), the palletizer and the hands-off pick script, the unit tests |
| Tanay Ranjan | Integration | `robot/.../sort_pile.py`: the loop that scans, picks, recovers and empties the whole pile; per-colour stacks |
| Shiva Rudra | Perception | `perception/`: the colour models, the batch comparison and the sim-trained colour work |
| Kondra Varun Teja | Perception | `classification/real_bricks/`: the real-brick classifier |
| Ahmet Sinan Erdem | Vision and simulation | with the team's simulation work in `simulation/` |

The Franka software in `franka/` is Franka Robotics' own (Apache 2.0, licences included); `simulation/factory_line/assets/franka_fr3/`
is the FR3 model from MuJoCo Menagerie (its licence is included).

On the robot, the smoothing cut the peak jerk of real moves from 49-461 to 0.3-2.0 rad/s3 with the hand held within 0.01 mm of the line.

## Where the code came from

| Folder | Source (branch of the team repository) |
|---|---|
| `robot/` | `submission` (`fr3_lego_sorter/src/fr3_pose_pick`): Ameed's pose pick, place, smoothing and palletizing, with Tanay's `sort_pile.py` |
| `perception/` | `colour-model-and-batch-compare` (Shiva), overlaid with the newer files of `submission` |
| `classification/real_bricks/` | `submission` (`Classify_phy_bricks`, Varun) |
| `calibration/results/` | `worktree-pose-pick-place` (`data/calibration`) |
| `calibration/handeye_tool/` | the separate `franka-handeye` project (its git history is not included) |
| `franka/` | the Franka packages built in the team workspace (upstream sources, no git metadata). `olvx_descriptions_module` is left out: nothing here uses it |
| `simulation/factory_line/` | the team's Isaac Sim ACME line, with the FR3 model from MuJoCo Menagerie kept in `assets/franka_fr3/` |
| `simulation/brick_scenes/` | `colour-model-and-batch-compare` (`sim/`) |

## Notes

- `perception/perception/` (the SAM3 detection code) has that name because the camera pipeline looks for it by that relative path.
- The robot settings are the ones of the `submission` branch (tuned on 27 Oct). They differ from the 26 Oct values: for example the
  gentle first close of the hand (`grasp_force_first`) is switched off there.
- Check the recorded poses in `robot/fr3_pose_pick/config/poses.yaml` against the current table before the arm moves.
