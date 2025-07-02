# WindAll
WindAll: Optimised filament winding including for non-axisymmetric shapes.

# User guide
##Installation
Full utilisation of WindAll requires the installation of 4 components; the core WindAll functions and scripts, FreeCAD to run these, Gmsh for meshing functions and Code Aster to perform the structural analysis. In order to use FreeCAD with Code Aster in order to use the structural analysis and optimisation functions of WindAll then a custom version of FreeCAD must be used. However the path generation and post-processor functions of WindAll can be used with a standard version of FreeCAD without Code Aster. Specific instructions for installation of the components is given below.

###WindAll
The WindAll functions are a series of Python scripts which are intended to be run within FreeCAD. They do not require installation, they can simply be downloaded or cloned from:

[WindAll repository](https://github.com/AMRC-Composites/WindAll)
This contains the folder “freecadmacros” which contains the code and “freecadfiles” which contains some example files. Within FreeCAD, point the “User macros location” (found in Macro>Macros…  menu) to the location of the WindAll>freecadmacros folder.

###FreeCAD
FreeCAD is a parametric 3D CAD modelling package. Standard, release versions of FreeCAD can be downloaded (for Windows, Mac or Linux) from:

[FreeCAD official release](https://www.freecad.org/downloads.php)
As stated above, this allows the core functions of WindAll to be performed, but not the structural analysis or optimisation. In order to do this then it must be built from source using  version which includes support for Code Aster and for composite materials. Instructions for how to do this are given here:

[FreeCAD wiki](https://wiki.freecad.org/Compiling)

HOWEVER, instead of cloning the official FreeCAD repository to build, you must instead clone this modified version of FreeCAD:
[FreeCAD-CodeAster Integration fork](https://github.com/AMRC-Composites/FreeCAD-CodeAster.git)

###Gmsh
Gmsh is a meshing application and is easily installed (for Windows, Mac or Linux) after downloading from its official website:

[Gmsh official website](https://gmsh.info/)

On installing then the location of Gmsh executable must be set in FreeCAD in the Edit>Preferences ...>FEM>Gmsh menu, NOTE: To see the FEM options in the Preferences menu, you must have first opened the FEM workbench.

### Code Aster

Code Aster is a Finite Element Analysis (FEA) solver. It is not currently supported by the official  version of FreeCAD, hence the need to use the custom version described above. The official website for Code Aster is here:

[Code Aster official website](https://code-aster.org/)

However this is arduous to install and only appears to support version 20.04 of Ubuntu Linux. A simpler way to install on Linux is to follow these steps:
1. Install Nix package manager from: [NixOS official website](https://nixos.org/download/)
2. Run the command:
```
nix build github:AMRC-Composites/nix-codeaster --extra-experimental-features nix-command --extra-experimental-features flakes
```
This will build Code Aster and all of its dependencies from source, so this will take some time (and considerable processing power and memory) the first time it is done. If updates are run again in future then they will be much quicker. Nix emphasises reproducibility and reliability, so it is also possible to lock this to a specific Git commit, such as:
```
nix build github:AMRC-Composites/nix-codeaster/9128e25bd7edeb6ab4b10927dd89399497133b0a --extra-experimental-features nix-command --extra-experimental-features flakes
```
3. This will create a (link to the) result folder in the Home directory. You must now enter the location of the Code Aster executable in the Edit>Preferences>FEM>CodeAster menu as shown below. NOTE: in order to access the FEM options within Preferences you must first have opened the FEM workbench in that FreeCAD session.
    <img src="/.github/images/CApreferences.png" width="800"/>
    
#Here be dragons
After this point, the user guide is just a work in progress to be updated later!
<img src="/.github/images/dragon.png" width="200"/>
##Generating paths
###Axisymmetric case
* Draw geometry NOTE: must align y axis to machine
* Draw drivecurve
* Mesh NOTE must rename Mesh object to not include special characters in name such as ()
* gui_stockcirclecreatortask with mesh and drivecurve selected
* Select stockcircles and load up gen_constthickonstockcirclestask, then click Apply several times, preview what the thickness will be using See Onion Layers
* You can set the current splay angle back to 18, and increase 
Desired thickness (*and* DThickness lower) in order to build more layers 
over the top of the current layer.
Set the windingprefix to, say, "m" to distinguish it in the next stage
* Select planwindings and run gui_genpathfromplanwindings
* Do the planwindings in batches selected by the windingprefix
and set the thickness offset to the normal offset, and Output windings 
to save to a separate folder for the layer 
To cope with the wiggles, there is a thickness smoothing applied of 3.6, 
which is 3 repeats, with an average weighting of 0.6 between the normal 
and the averaged normal of the forward and back normals
Use the Negate Z option to wind in the ‘normal’ direction, I.e tow being deposited on the top of the mandrel.
Se a thinning tolerance of at least 1 to get sensible results.
* Select outputwindings and run gui_postprocesswindings.
* Toolpath output is in fillwind11.src

###Non-axisymmetric case
* guidirectedgeodesicpaths with mesh and drive curve most basic function, other functions for iterating and multi drive curves built on this.
