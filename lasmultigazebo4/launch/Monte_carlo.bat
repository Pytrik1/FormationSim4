@echo off                           //Turn off screen text messages
:loop                               //Set marker called loop, to return to
start "N_nexus_world" "C:\Wicked Article Creator\Wicked Article Creator.exe"  //Start program, with title and program path 
timeout /t 1200 >null               //Wait 20 minutes
taskkill /f /im "roslaunch" >nul   //Image name, e.g. WickedArticleCreator.exe, can be found via Task Manager > Processes Tab > Image Name column (look for your program)
timeout /t 7 >null                  //Wait 7 seconds to give your prgram time to close fully - (optional)
goto loop                           //Return to loop marker
