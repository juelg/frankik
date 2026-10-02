# Bundled robot models

Kinematics-only MuJoCo MJCF models that ship with `frankik` for the numerical
(`PinocchioKinematics`) solver. Select them via `frankik.RobotType`:

| `RobotType` | file        | origin (MuJoCo Menagerie) | `tcp_frame`       | `base_frame` | dof |
|-------------|-------------|---------------------------|-------------------|--------------|-----|
| `PANDA`     | `panda.xml` | `franka_emika_panda`      | `attachment_site` | `link0`      | 7   |
| `FR3`       | `fr3.xml`   | `franka_fr3`              | `attachment_site` | `base`       | 7   |

## Modifications

The files are derived from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
models. To keep the package small, all meshes, materials, textures and geoms were
removed; joints, joint limits, inertials, sites, actuators and keyframes are unchanged.

`panda.xml`: the Menagerie model places `attachment_site` inside a joint-less child
body `attachment` that is rotated by 135° about z. Pinocchio's MJCF parser does not
export sites of such bodies, therefore the site is attached directly to `link7`.
Its orientation was aligned with the Franka flange frame (libfranka `O_T_EE`
without end effector), identical to the `attachment_site` of the FR3 model, so
both robots share the same TCP convention (`FrankaKinematics.FrankaHandTCPOffset`
transforms it to the Franka Hand TCP).

Note that the joint limits in these files are the Menagerie/URDF limits of the
robots. `frankik.q_min_fr3`/`q_max_fr3` used by the analytical solver are the
more conservative limits of libfranka.

## License

The MuJoCo Menagerie Franka models are licensed under the Apache License 2.0
(Copyright 2023 DeepMind Technologies Limited / Franka Robotics GmbH), see
https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_fr3/LICENSE and
https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/LICENSE.
