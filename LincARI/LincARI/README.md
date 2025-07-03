# LincARI
ROS noetic packages for the ARI robot in L-CAS.


## Usage

### Setup your environment

1. Make sure you have VSCode installed: https://code.visualstudio.com/download
2. Make sure you have git installed: https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
3. Make sure you have the `Docker` and the `Dev Containers` extension in VSCode installed and working: https://code.visualstudio.com/docs/containers/overview and https://code.visualstudio.com/docs/devcontainers/containers
    * ensure docker is working, i.e. try `docker run --rm hello-world` and check it succeeds for your user
4. The docker image used to provide the Development Container is provided by the [L-CAS](https://lcas.lincoln.ac.uk) Container Registry. You must log in to use it. For simple read access, the username and password is public and is username `lcas`, password: `lincoln`. So, to log in do `docker login -u lcas -p lincoln lcas.lincoln.ac.uk` (you should only have to do this once, as the credentials should be cached unless your home directory is wiped).

### Open in VSCode

#### 1. Clone the repository
##### ➡️ If you are cloning for the first time:
❗**If you are on a Windows PC the following two additional steps are required:**

   - Open a terminal(e.g., window's powershell), type `git config --global core.autocrlf false` and press Enter
   - Make sure docker is running by launching the docker desktop application

Then:
1. Open a terminal (e.g., window's powershell)
2. Execute the following command: `git clone https://github.com/LCAS/LincARI.git`

##### ➡️ If you already have a local copy:
1. Open a terminal (e.g., window's powershell)
2. Locate and move to the LincARI folder on your system; e.g. `cd /home/computing/LincARI`
3. Pull the latest changes: `git pull`

#### 2. Open the ROS2 container
The system configuration for writing and executing ROS code is already setup for you in a Docker container which is directly importable into your VS Code workspace. This will avoid you executing many additional steps the you will need instead to perform if you want to run it on your PC.

1. Open VS Code (e.g., Alt+F2, type `code`, press Enter)
2. Click on the blue icon in the bottom left corner 
     <img width="59" alt="image" src="https://github.com/francescodelduchetto/RBT1001/assets/7307164/adc84af7-daa9-4470-a550-06e017a5cf2c">

3. Select "Open Folder in Container..."
4. Select and Open the folder LincARI

The first time you perform this, it will download and compile the ROS system and all the dependencies; therefore, it will take quite some time. 

#### 3. Connect to VNC (Graphical Interface)


1. Click on the "Port" in VSCode, find the "novnc" port, right click on it to open the menu, and then choose either "Open in Browser" to open it outside of VSCode or "Preview in Editor" to have it open within VSCode:

   <img width="735" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2b0bdfa9-07ea-4238-a0b9-dd2dc8f4c111">

2. (recommended) Set the dekstop scaling by clicking on the settings cog and choose scaling mode "Remote Resizing" if it's not set

   <img width="292" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2d9bc88e-7319-4723-968a-0aa08db026ef">

3. click on "Connect":

   <img width="455" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/ddc224eb-5980-4d9a-994e-b05aa1e9fc1d">



