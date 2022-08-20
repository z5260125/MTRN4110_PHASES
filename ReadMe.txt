DaBaby Team Tutorial

There are THREE worlds in our submission - one for the combination of Phases A, B and C, one for additional features and one specifically for the alternate robot.

How to run Phases A to C

1. Replace WEBOTS_DIRECTORY with the proper path to webots.exe on line 314 of the "DABABY_MTRN4110_PhaseC.py" file
2. Go to directory of DABABY_MTRN4110_PhaseD in Anaconda prompt "cd path/to/webots"
3. Activate virtual environment - "conda activate mtrn4110" (instructions for Anaconda Prompt/Python installation here: https://github.com/drliaowu/MTRN4110_Python_Tutorials/blob/master/Tutorial-1-Installing-python.md)
4. Run phase C file in terminal - "python DABABY_MTRN4110_PhaseC.py"

PhaseC will run and each figure needs to be closed before the next one appears. Upon finish, DABABY_MTRN4110_PhaseD world will open and run Phases B and A automatically.


How to run Phase D additional features

1. Open worlds/DABABY_MTRN4110_PhaseDAdditional.wbt
2. Use Keyboard to enter prompts to choose an additional feature to run:
  
  2.1. Keyboard Mode
    2.1.1. Press 1 on keyboard
    2.1.2. Use up, down, left and right keys to move forward, backward, and to turn left and right respectively
    2.1.3. Use the space bar or equivalent ' ' to stop moving
    2.1.4. Use the q key to stop moving and quit the Keyboard Mode
    2.1.5. Use w, s, a and d keys to move forward, backward, and to turn left and right respectively
    2.1.6. The robot will continue to move in a direction for as long as the key is held


  2.2. Keyboard Sensor Mode
    2.2.1. Press 2 on keyboard
    2.2.2. Press a number between 0 and 4 on keyboard to set targetRow where 0 is the topmost row and 4 is the bottom row
    2.2.3. Press a number between 0 and 8 on keyboard to set targetCol where 0 is the leftmost column and 8 is the rightmost column
    2.2.4. Use up, left and right keys to move forward, and to turn left and right respectively
    2.2.5. Use w, a and d keys to move forward, and to turn left and right respectively
    2.2.6. Use the q key to stop moving and quit the Keyboard Sensor Mode
    2.2.7. The robot will move exactly one square forward or turn exactly 90 degrees per input
    2.2.8. The robot will take 10 seconds between moves to check surrounding walls - console will indicate when it is ready for next input
    2.2.9. The robot will print map with updated found walls to console after each motion step
    
    
  2.3. Keyboard Sensor Mode
    2.3.1. Press 3 on keyboard
    2.3.2. Press a number between 0 and 4 on keyboard to set targetRow where 0 is the topmost row and 4 is the bottom row
    2.3.3. Press a number between 0 and 8 on keyboard to set targetCol where 0 is the leftmost column and 8 is the rightmost column   
  
  
  
  
  
  
  
  
  
  
  
  
  2.4. Keyboard Mapping and Pathfinding Mode
    2.4.1. Press 4 on keyboard
    2.4.2. Press a number between 0 and 4 on keyboard to set targetRow where 0 is the topmost row and 4 is the bottom row
    2.4.3. Press a number between 0 and 8 on keyboard to set targetCol where 0 is the leftmost column and 8 is the rightmost column
    2.4.4. Use up, left and right keys to move forward, and to turn left and right respectively
    2.4.5. Use w, a and d keys to move forward, and to turn left and right respectively
    2.4.6. Use the q key to stop moving and quit the Keyboard Mapping and Pathfinding Mode
    2.4.7. The robot will move exactly one square forward or turn exactly 90 degrees per input
    2.4.8. The robot will take 10 seconds between moves to check surrounding walls - console will indicate when it is ready for next input
    2.4.9. The robot will print map with updated found walls to console after each motion step
    2.4.10. The robot will display the minimum number of adjacent squares the robot will have to travel through to reach the target position
    
  2.5. Seek Greeen Square Mode
    2.5.1. Press 5 on keyboard, starting a timer
    2.5.2. Use up, down, left and right keys to move forward, backward, and to turn left and right respectively
    2.5.3. Use w, s, a and d keys to move forward, backward, and to turn left and right respectively
    2.5.4. Use the q key to stop moving and quit the Seek Green Square Mode
    2.5.5. The robot will continue to move in a direction for as long as the key is held
    2.5.6. Using the movement keys fill as much of the camera's vision with green as possible in the smallest time possible
    
  2.6 Quitting program - to end the program simply press the 'end' key on your keyboard
    


How to run Phase D with replaced robot.

1. Open worlds/DABABY_MTRN4110_KailaBot.wbt
2. KailaBot will automatically run based on the instructions in the "Plan.txt" two directories above the robot, so changing the contents of this file will change the instructions that KailaBot follows.
3. KailaBot simply performs the basic parts of Phase A, namely moving forward one square, turning 90 degrees both left and right, reading in instructions from a text file and being able to follow them
