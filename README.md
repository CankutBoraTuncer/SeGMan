## Overview
<p align="center">
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/SeGMan.svg" alt="SeGMan"/>
</p>

In this paper, we present SeGMan, a hybrid motion planning framework that integrates sampling-based and optimization-based techniques with a guided forward search to address complex, constrained sequential manipulation challenges, such as pick-and-place puzzles. SeGMan incorporates an adaptive subgoal selection method that adjusts the granularity of subgoals, enhancing overall efficiency. Furthermore, proposed generalizable heuristics guide the forward search in a more targeted manner. Extensive evaluations in maze-like tasks populated with numerous objects and obstacles demonstrate that SeGMan is capable of generating not only consistent and computationally efficient manipulation plans but also outperform state-of-the-art approaches.
https://sites.google.com/view/segman-lira/

## Installation

### Prerequisites
Ensure you are using **Ubuntu 20.04** with **Python 3.8.10 or later**.

### Installation Steps

1. **Update and install dependencies**
   Open a terminal and run:
   ```bash
   sudo apt update && sudo apt install -y python3 python3-pip python3-venv
   ```

2. **Verify Python version**
   Ensure you are using Python 3.8.10 or later:
   ```bash
   python3 --version
   ```
   If you need to install Python 3.8.10, you can do so with:
   ```bash
   sudo apt install -y python3.8
   ```

3. **Create a virtual environment (Optional but recommended)**
   ```bash
   python3 -m venv segman
   source segman/bin/activate
   ```

4. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

## Running the Project
To run the test files located in the `test` folder, first navigate to the folder:
   ```bash
   cd test
   ```
Then, execute any test file using:
   ```bash
   python filename.py
   ```

## Example Solutions
<div align="center">
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/maze.gif" width="250" height="250" /> 
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/wall-easy.gif" width="250" height="250" /> 
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/wall-hard.gif" width="250" height="250" />
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/two-blocks.gif" width="250" height="250" />  
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/four-blocks.gif" width="250" height="250" /> 
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/slot.gif" width="250" height="250" /> 
  <img src="https://github.com/CankutBoraTuncer/SeGMan/blob/main/visuals/lock.gif" width="250" height="250" /> 
</div>
