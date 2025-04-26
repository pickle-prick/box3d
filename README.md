# A Simple Physics Engine built for understanding how it work

## Build & Run

### Build

```bash
cargo build --release
```
it should work out-of-the-box on Mac and Windows, but on Linux you need to first install (required by egui):

`sudo apt-get install -y libclang-dev libgtk-3-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libxkbcommon-dev libssl-dev`

On Fedora Rawhide you need to run:

`dnf install clang clang-devel clang-tools-extra libxkbcommon-devel pkg-config openssl-devel libxcb-devel gtk3-devel atk fontconfig-devel`

### Run

```bash
./target/release/box3d
```

## Basic control (Demo)

### Camera
* Press and hold the Right Mouse Button to adjust the camera's yaw and pitch
* Scroll the Middle Mouse Button to zoom the camera in or out
* Press and hold the Middle Mouse Button to translate the camera left or right
### Cuboid
* Click on the cuboid body to apply a force to it
* Drag the contact ball to adjust the contact point

## Dependencies
* egui
* bevy

## Reference

### Physically Based Modeling
https://graphics.stanford.edu/courses/cs448b-00-winter/papers/phys_model.pdf

### Numerical Recipes in C.
https://www.cec.uchile.cl/cinetica/pcordero/MC_libros/NumericalRecipesinC.pdf

### ConstraintsDerivationRigidBody3D
https://danielchappuis.ch/download/ConstraintsDerivationRigidBody3D.pdf

### Distance Constraint
https://dyn4j.org/2010/09/distance-constraint/
https://dyn4j.org/2010/12/max-distance-constraint/

### Constrainted Rigidbody Simulation
https://www.toptal.com/game/video-game-physics-part-iii-constrained-rigid-body-simulation
