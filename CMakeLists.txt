CMAKE_MINIMUM_REQUIRED(VERSION 3.1...3.26)
PROJECT(cpp-interface-coppeliasim)
set (CMAKE_CXX_STANDARD 11)

################################################################
# INSTALL HEADERS
################################################################

INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim") 
