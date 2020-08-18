Maya Animation Rendering:
1. right click and set up rendering speed
2. Go to render setting (a small symbol)
```select render camera and frame range 
```select image format and the Frame/Animation ext to name_#.ext.
```notice the top of the window has the path to which your rendered images are saved 
3. from Modeling to Render field
4. Select Render > Render Sequence >  
```from the Rendering menu set to open the Render Sequence options window
5. Enable Add to Render View
6. Click Render Sequence

Note:
1. Before simulate need to manually delete all objects on the scene
2. The last line of your data scnariofile should not contain \n
3. Moving obstacle has minimum radiu 0.6
4. Need to modify the following in CrowdCmd.cpp(line 30-32)
const string TEMP_FILE_NAME = "YOUR PATH TO PROJECT LOCATION/CrowdsParameters/tempScene.txt";
const string ORIGINAL_FILE_NAME = "YOUR PATH TO PROJECT LOCATION/CrowdsParameters/original_file.txt";
MString parameterFile = "YOUR PATH TO PROJECT LOCATION/settings.ini";


