# This file is used when other components needs to use something provided by this DCL. 
# Provide any include paths and lib directories. Use /home/pwasows1/discode_ws/DCL/Hanoi/dist
# to point to 'dist' directory of current DCL, it'll be substituted during installation. 

# directory containing header files
SET(Hanoi_INCLUDE_DIR /home/pwasows1/discode_ws/DCL/Hanoi/dist/include)
INCLUDE_DIRECTORIES(${Hanoi_INCLUDE_DIR})

# directory containing libraries
SET(Hanoi_LIB_DIR /home/pwasows1/discode_ws/DCL/Hanoi/dist/lib)
LINK_DIRECTORIES(${Hanoi_LIB_DIR})

# list of libraries to link against when using features of Hanoi
# add all additional libraries built by this dcl (NOT components)
# SET(Hanoi_LIBS lib_1 lib_2)
# SET(ADDITIONAL_LIB_DIRS /home/pwasows1/discode_ws/DCL/Hanoi/dist/lib ${ADDITIONAL_LIB_DIRS})
