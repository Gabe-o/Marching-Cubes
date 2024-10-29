# Assignment 5
<img src="https://github.com/Gabe-o/Marching-Cubes/blob/main/Screenshot1.png" width="500">
<img src="https://github.com/Gabe-o/Marching-Cubes/blob/main/Screenshot2.png" width="500">

## Info
Rendering 3d functions using marching cubes

## Submission
### Exercise1.cpp
This is my main file for rendering the marching cubes algorithm.

The executable takes up to 8 args:

    1. Screen width     (default = 1400)
    2. Screen height    (default = 900)
    3. Step size        (default = 0.05)
    4. Min world xyz    (default = -5)
    5. Max world xyz    (default = 5)
    6. Isovalue         (default = 0)
    7. function         (default = 1) 
        a. 1 = y-sin(x)cos(z)
        b. 2 = x^2-y^2-z^2-z
        c. 3 = x^2+y^2+z^2
    8. bonus            (default = 1)
        a. 1 = bonus enabled: render as marching algorithm is running 
        b. 0 = bonus disabled: precompute entire mesh then render

To render y-sin(x)cos(z) with isovalue of 0:
`
./Exercise1.exe 1400 900 0.05 -5 5 0 1 1
`

To render x^2-y^2-z^2-z with isovalue of -1.5:
`
./Exercise1.exe 1400 900 0.05 -5 5 -1.5 2 1
`


#### marching_cubes function
Performs the marching cubes algorithm and generates the mesh based on the input params.

#### compute_normals function
Takes the mesh created by the marching_cubes function and calculates the normal for each face.

#### write_PLY function
Writes the vertices and normals to a .ply file.

#### marching_cubes_bonus function
Combines the marching_cubes, compute_normals, and write_PLY functions into a single file execept this takes a refrence to a ThreadSafeQueue of faces which the vertices for each face are pushed too once they are ready. These can then be dequeued inside the render loop to draw the mesh as the marching cubes algorithm is functioning.

### Cam Controls.hpp
This contains my logic for generating the V matrix based on the camera's desired position. I flip the controls and up vector when you move over the poles so the camera doesn't flip. Click and drag with left mouse to move. Arrow keys to move forward and back. 

### Axes.hpp
This came from the Lecture 14 Misc Files on the OWL page. I used it to render the axes. 

### shader.hpp
This came from the Lecture 14 Misc Files on the OWL page. I used it to load my shaders.

### TriTable.hpp
This came from the Assignment 5 page. I used it's lookup table in the marching cubes algorithm

### DiffuseShader.vertexshader
Phong vertex shader.

### DiffuseShader.fragmentshader
Phong fragment shader.

### Screenshot1.png & function1.ply
Screenshot and .ply file of the output for y-sin(x)cos(z) with isovalue of 0:
```
./Exercise1.exe 1400 900 0.05 -5 5 0 1 1
```

### Screenshot2.png & function2.ply
Screenshot and .ply file of the output for x^2-y^2-z^2-z with isovalue of -1.5:
```
./Exercise1.exe 1400 900 0.05 -5 5 -1.5 2 1
```

# Windows Build Enviroment Setup
1. Install MSYS2 (https://www.msys2.org/)
2. Open MSYS2

    a. Install pacman: `pacman -Syu`
    
    b. Install MinGW: `pacman -S mingw-w64-x86_64-toolchain`

    c. Install Freegult: `pacman -S mingw-w64-x86_64-freeglut`

    d. Install Glew: `pacman -S mingw-w64-x86_64-glew`

    e. Install GLFW `pacman -S mingw-w64-x86_64-glfw`

    f. Install glm `pacman -S mingw-w64-x86_64-glm`

3. Add `C:/msys64/mingw64/bin` to user PATH variable 
4. Add `C:/msys64/mingw64/include/**` to your includePath in editor

### VS Code Setup
I use VS Code as my editor to get that working properly I had to install the C/C++ extension and update my config files as follows:
#### .vscode/task.json
```json
{
    "tasks": [
        {
            "type": "cppbuild",
            "label": "C/C++: g++.exe build active file",
            "command": "C:\\msys64\\mingw64\\bin\\g++.exe",
            "args": [
                "-fdiagnostics-color=always",
                "-g",
                "${file}",
                "-o",
                "${fileDirname}\\${fileBasenameNoExtension}.exe",
                "-lfreeglut",
                "-lglew32",
                "-lopengl32",
                "-lglfw3",
            ],
            "options": {
                "cwd": "${fileDirname}"
            },
            "problemMatcher": [
                "$gcc"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "detail": "Task generated by Debugger."
        }
    ],
    "version": "2.0.0"
}
```

#### .vscode/c_cpp_properties.json
```json
{
    "configurations": [
        {
            "name": "Win32",
            "includePath": [
                "${workspaceFolder}/**",
                "C:/msys64/mingw64/include/**"
            ],
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ],
            "windowsSdkVersion": "10.0.22621.0",
            "compilerPath": "C:/msys64/mingw64/bin/g++.exe",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "windows-gcc-x86"
        }
    ],
    "version": 4
}
```

#### Building
Open the file to be built, then from the top bar select `Terminal > Run Build Task`
