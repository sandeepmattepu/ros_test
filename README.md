# Documentation for solutions of tasks
## Compilation Erros
### Error 1 : "Could not find package joy" produced by CMakeLists.txt file when compiling the package
#### Solution :
Remove package name "joy" from `find_package` block in [CMakeLists.txt](./CMakeLists.txt) file.
#### Approach :
1. Tried to compile package using `catkin_make` and it produced error in terminal stating the error.
2. Looked at all the source code files if any part of the code is using package called "joy" and we found none.
3. We removed the "joy" package and recompiled. The package missing error is fixed.

### Error 2 : "Has no member named 'turn' in inf_main.cpp" and "Has no member named 'forward' in inf_main.cpp" produced when compiling the package
#### Solution :
Change all the lines in [inf_main.cpp](./src/inf_main.cpp) file where members of `turtle` variable are being accessed with dot operator(Eg:- `turtle.forward(3)`), replace them with arrow operator(EgL- `turtle->forward(3)`)
#### Approach :
1. Tried to compile package using `catkin_make` and it produced errors in the terminal stating that problem is from  [inf_main.cpp](./src/inf_main.cpp) file.
2. Looked at data type of `turtle` variable which is `std::shared_ptr<>` type.
3. Replaced all the dot operators with arrow operators.
4. Recompiled the project and this error got fixed.

### Error 3 : "Requires the 'velocities' arg to be set" produced when trying to launch inf.launch file
#### Solution :
Change the line `<param name="velocity" value="$(arg velocities)" />` to `<param name="velocity" value="$(arg velocity)" />` in [inf.launch](./launch/inf.launch) file
#### Approach :
1. Tried to launch [inf.launch](./launch/inf.launch) in terminal but it shows error in the terminal stating that it requires the `velocities` arg to be set.
2. By fixing the typo from `velocities` to `velocity` in [inf.launch](./launch/inf.launch) file the error is fixed
