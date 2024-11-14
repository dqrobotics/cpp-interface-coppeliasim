CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(dqrobotics-interface-coppeliasim)
set (CMAKE_CXX_STANDARD 14)

################################################################
# INSTALL HEADERS
################################################################

INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim") 
