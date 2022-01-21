# FRC 2022

[![CI](https://github.com/SummitRobotics/FRC2022/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/SummitRobotics/FRC2022/actions/workflows/main.yml)

Team 5468's 2022 FRC robot code. This code is written in Java and is based off of WPILib's Java control system and utilizes a command based system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2022-Public.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2022-Public` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Variable Naming Conventions
- TODO

## Branch Naming Conventions
- clean_##### - A branch for cleaning up code
- clean_NAME - A branch for cleaning up code
- dev_##### - A branch for rapid development (Ex. At comps)
- dev_NAME - A branch for rapid development
- fix_NAME - A branch for making thoughout fixes
- hotfix_NAME - A branch for hotfixes
- \#\#\#\#\# - Number in the format MonthDayIncrement (Ex. for the second cleanup branch on Jan 20 (clean_01202))
