# helix_ws



https://github.com/user-attachments/assets/9978f1c3-46bd-4590-b03c-bf07871c407f

This project develops a Model Predictive Control (MPC) strategy for energy conservation and time optimization on the Jackal, an autonomous vehicle by Clearpath Robotics. A Kalman Filter integrates data from the Jackal's Lidar and IMU sensors. The MPC algorithm guides the Jackal to a reference point, validating the energy-saving strategy through various MPC parameter configurations.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contact](#contact)

---

## Features

✅ Kalman Filter for pose and velocity data

✅ Test process for vehicle's dynamic state

✅ Implement Model Predictive Control (MPC) to navigate the vehicle through multiple reference points, optimizing energy conservation and travel time.

---

## Installation

### Prerequisites

- Docker

### Create the Docker env

#### Create a Working Directory

On your host machine, create a directory to store the Dockerfile:

```bash
mkdir jackal_ws
cd jackal_ws
```

#### Download the Dockerfile

Download the Dockerfile by click Download raw file on the right-up side of the screen.

Move the Dockerfile to the jackal_ws

```bash
mv ~/Downloads/Dockerfile ~/jackal_ws/
```

#### Create the Docker Contaienr

```bash
cd ~/jackal_ws
docker build -t noetic_docker .
```

#### Verify the Image

```bash
docker images
```

You should see noetic_docker in the list of images.

#### Run the Container

```bash
docker run -it noetic_docker
```

#### Clone the Code inside the Container

```bash
cd /root/helix_ws
git clone https://github.com/zjiang11/helix_ws.git
```


---

## Usage

### Build Program

```bash
cd ~/helix_ws
catkin_make
source devel/setup.bash
```

