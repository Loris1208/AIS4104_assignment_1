# Licence texts deployed beside the robot descriptions

One row per package this project deploys. The archive digest is the `HASH SHA256=` pinned for that
archive in the root listfile.

| Deployed package | Archive | Archive digest | Package manifest declares | Text that travels | Copyright line that text holds |
| --- | --- | --- | --- | --- | --- |
| `kuka_kr6_support` | `ros-industrial/kuka_experimental` at `8d9292b04a22628b1b78d989e2ddd3abb913bf92` | SHA256 `02f299d967868fb32022617429ddede3a197fb8f18e09b88f88732e11a78bf79` | `<license>BSD</license>` | `apache-2.0.txt` | None. The text carries the unfilled appendix placeholder `Copyright {yyyy} {name of copyright owner}` and names no holder. |
| `kuka_resources` | `ros-industrial/kuka_experimental` at `8d9292b04a22628b1b78d989e2ddd3abb913bf92` | SHA256 `02f299d967868fb32022617429ddede3a197fb8f18e09b88f88732e11a78bf79` | `<license>Apache2</license>` | `apache-2.0.txt` | None. The text carries the unfilled appendix placeholder `Copyright {yyyy} {name of copyright owner}` and names no holder. |
| `ur_description` | `UniversalRobots/Universal_Robots_ROS2_Description` tag `4.3.1` | SHA256 `3532a25c9942bedcdfe41c845be6d144bbbbcd76f179f14a53a3e7b6972ae72d` | `<license file="LICENSE">BSD-3-Clause</license>` | `bsd-3-clause.txt` | None. The text opens at "Redistribution and use in source and binary forms" and names no holder. |
| `lbr_med14_r820_description` | `lbr-stack/med14_r820_description` tag `v2.5.0` | SHA256 `edb596d3e2b7f07f5b8f66ef68cd3ccfef0212e98efe25742867cfbe653deb5a` | `<license>Apache-2.0</license>` | `apache-2.0.txt` | None. The text carries the unfilled appendix placeholder `Copyright {yyyy} {name of copyright owner}` and names no holder. |

## The two texts

| File | Copied from | SHA256 |
| --- | --- | --- |
| `apache-2.0.txt` | `LICENSE` at the root of the `ros-industrial/kuka_experimental` archive | `6dc0e068dcf3a5bc8e054205b85b7720e1d49265bbc64bf515d2cf79197df69a` |
| `bsd-3-clause.txt` | `LICENSE` at the root of the `UniversalRobots/Universal_Robots_ROS2_Description` archive | `143077f9f6a8de5a99f78c2c2472801fb53cf240277ef661642f079b9acdfff8` |

Each is a byte-for-byte copy of the file named above. Neither was edited, reflowed or completed:
the Apache-2.0 appendix placeholder stands unfilled exactly as its upstream publishes it.

## What the upstreams publish

- `kuka_kr6_support` declares `BSD` and the `ros-industrial/kuka_experimental` archive publishes no
  BSD text anywhere. The only licence text in that archive is the Apache-2.0 at its root, and that
  is the text deployed for this package. The declaration and the deployed text do not name the same
  licence.
- `kuka_resources` declares `Apache2` and ships no licence file of its own. The archive's root
  Apache-2.0 text is deployed for it; declaration and text agree.
- `ur_description` declares `BSD-3-Clause` and ships the matching text at its archive root.
  Its manifest carries a second live entry,
  `<license file="meshes/ur20/LICENSE.txt">Universal Robots A/S’ Terms and Conditions for Use of
  Graphical Documentation</license>`, which names `meshes/ur20`. This project deploys
  `meshes/ur3e` and no other mesh directory from that package.
- `lbr_med14_r820_description` declares `Apache-2.0` and its archive holds no licence file at any
  path. The Apache-2.0 text carried here is deployed for it.

This project's own `LICENSE` is not among these and is not deployed.
