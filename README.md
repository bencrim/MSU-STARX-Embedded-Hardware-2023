# MSU STARX Embedder Hardware Team GitHub 2023

![](https://www.egr.msu.edu/sites/default/files/2021-09/STARX_One_Color_web_Logo.png)

Below is a beginner walkthrough on how to install the Arduino IDE and the libraries we will be using in it.

1. Download and install Arduino IDE (https://www.arduino.cc/en/software)
2. Download the GitHub repository and remember where you download it to. (https://github.com/UncommonCandy/MSU-STARX-Embedded-Hardware-2023) This can be done by clicking the green "Code" button above and downloading the zip file.
3. Locate the GitHub zip file you just downloaded and extract it somewhere you can access. Then open the extracted GitHub folder and navigate to the libraries folder under STARX Arduino "...\MSU STARX Revised GitHub\STARX Arduino\libraries".
4. Locate the file location for the Arduino IDE you just installed, then open the libraries folder. Mine defaulted to my C Drive under Program Files (x86). "C:\Program Files (x86)\Arduino\libraries".
5. With both library folders open, copy all folders from the "...\MSU STARX Revised GitHub\STARX Arduino\libraries" folder into the "...\Arduino\libraries" folder. There should already be some folders in the "...\Arduino\libraries" folder, so when it prompts you to replace them, click yes.
6. After waiting for the folders to copy over, open the Arduino IDE and then navigate to the "Manage Libraries" window under "Tools" at the top of the IDE. This might take time to load.
7. Once the Library Manager finishes loading, change the type to "Updatable" in the "Type" dropdown menu in the top left. Update any libraries that need to be updated. This process can take some time.
8. Now you should have all the libraries currently in use by the team. Any new libraries added to the GitHub will have to be added to your Arduino library folder (Steps 4-6).
