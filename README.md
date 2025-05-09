# 706-Project-1

# Project File Descriptions
- globals.h
   Shared global variables, definitions, and pin settings.

- moving_logic.h / moving_logic.cpp
   Functions that control robot movement (forward, reverse, turn, etc.).

- sensors.h / sensors.cpp
   Functions to read sensor data (ultrasonic, analog, gyro, battery).

- utilities.h / utilities.cpp
   Helper functions (LED flashing, speed smoothing).

- serial_command.h / serial_command.cpp
   Reads and processes serial commands to control the robot.

- state_machine.h / state_machine.cpp
   Finite state machine managing robot states (initialising, running, stopped).

- main.cpp
   Main entry point that sets up hardware and runs the state machine loop.

# Arduino Mega Project - Setup & Development Guide

## 1. Setting Up PlatformIO in VS Code
To develop and upload code for the Arduino Mega using VS Code, follow these steps:

### Install Required Software
1. Install Visual Studio Code (VS Code)
   [Download VS Code](https://code.visualstudio.com/)

2. Install PlatformIO Extension
   - Open VS Code
   - Go to Extensions (`Ctrl + Shift + X`).
   - Search for PlatformIO IDE
   - Click Install

3. Restart VS Code to apply changes.

---

## 2. Cloning the GitHub Repository
To get the latest project files on your computer:

### Clone the Repository (First Time Only)
1. Open VS Code and its Terminal (`Ctrl + ~`).
2. Navigate to a folder where you want to store the project:
   
   cd path/to/your/folder
   
3. Clone the repository:
   
   git clone https://github.com/fooooooop/706-Project-1.git
   
4. Enter the new project folder:
   
   cd 706-Project-1


---

## 3. Opening & Running the Project in PlatformIO
1. Open VS Code.
2. Click File > Open Folder... and select the `706-Project-1` folder.
3. PlatformIO should detect the project automatically.

### Build & Upload Code to Arduino Mega
1. Plug in your Arduino Mega via USB.
2. Open the PlatformIO Toolbar (Alien Icon 🛸 on the left).
3. Click "Build" (Checkmark ✔️) to compile the code.
4. Click "Upload" (Arrow ⬆️) to flash the code onto the board.
5. (Optional) Open the Serial Monitor to see logs:
   - Click the Monitor Icon (or `Ctrl + Alt + M`).

---

## 4. Pulling the Latest Code from GitHub
Before making changes, sync your local project with GitHub:


git pull origin main


>  Always pull before starting work to avoid conflicts!

---

## 5. Reset your code back to the latest change from Github

git reset --hard HEAD

---

## 6. Making Changes & Pushing to GitHub
Once you've made changes:

1. Check what’s changed:
 
   git status
 
2. Stage all changes:

   git add .

3.Commit the changes:

   git commit -m "Description of what you changed"

4. Push the changes to GitHub:

   git push origin main


---


