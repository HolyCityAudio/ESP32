#!/bin/bash

######## get a number within a given range
# $3 is the prompt, $1 is the minimum value, $2 is the maximum value
### e.g. getNumRange 2 5 will ask you for a number between 2 and 5
# validated input is returned in global variable getNumRangeValue
PORT=/dev/ttyS4
BAUD=115200
EDITOR="/mnt/c/Program Files (x86)/Notepad++/notepad++.exe"

######### Ask Function #########
ask()
{
    echo
    while true; do
        if [ "${2:-}" = "Y" ]; then
            prompt="Y/n"
    	    default=Y
	elif [ "${2:-}" = "N" ]; then
	    prompt="y/N"
    	    default=N
    	else
	    prompt="y/n"
    	    default=
    	fi
	# Ask the question
        echo "$1 [$prompt] "
	read REPLY
	# Default?
	if [ -z "$REPLY" ]; then
	    REPLY=$default
    	fi
	# Check if the reply is valid
	case "$REPLY" in
	    Y*|y*) return 0 ;;
    	    N*|n*) return 1 ;;
    	esac
    done
}
######### End Ask Function #########

############# validate input - check $1 against tr control string $2

validateInput()
{
    # Comb out invalid characters from input and assign to new variable
    export VAR_CLEAN="$(echo "$1" | tr -cd "$2" | sed 's/\[//' | sed 's/\]//' )"
    # if the before and after are the same, then there were no bad characters
    if [ "${VAR_CLEAN}" = "$1" ]; then
       echo 0
    else
       echo 1
    fi
}

############# end of validate input

getNumRange()
{
     validInput="n"
     while [ "$validInput" = "n" ]; do
         echo "$3 ($1 - $2):"
         read num
         if [ "$num" != "" ]; then
             echo
             if [ $(validateInput "$num" "0123456789") -ne 0 ]; then
                 printf "\nSorry, enter numbers only.\n"
	     else
	         # echo "You entered: $num"
		 if [ "${num}" -ge "$1" ]; then
		     if [ "${num}" -le "$2" ]; then
		        validInput="y"
		     else
                          echo "Sorry, input was not in the range $1 to $2"
                      fi
                 else
                     echo "Sorry, input was not in the range $1 to $2"
                 fi
             fi
         fi
    done
    getNumRangeValue="$num"
}


########## Begin menuLoop #################
menuLoop()
{
    while [ true ]; do
	printf "============================ Main Menu ============================\n"
	printf "                                                                   \n"
	printf "    1) Edit the $filename.dsp file                                 \n"
        printf "    2) Edit main.cpp                                               \n"
        printf "    3) Edit $filename.cpp                                               \n"
        printf "    4) Edit any file                                               \n"
        printf "    5) Execute idf.py fullclean                                    \n"
	printf "    6) Build C++ code only                                             \n"
	printf "    7) Compile DSP code only                                             \n"
	printf "    8) Build and upload C++ code, then monitor                         \n"
	printf "    9) Monitor                         \n"
	printf "    10) make menuconfig                                             \n"
	printf "    11) Save as reference                                           \n"
	printf "    12) Exit                                                        \n"
	printf "                                                                   \n"
	printf "===================================================================\n"

	getNumRange 1 12 "Choose a menu selection"
	get_num="$getNumRangeValue"

	if [ "$get_num" = 1 ]; then
            "$EDITOR" main/$filename.dsp &
        elif [ "$get_num" = 2 ]; then
            "$EDITOR" main/main.cpp &
        elif [ "$get_num" = 3 ]; then
            "$EDITOR" main/$filename.cpp &
        elif [ "$get_num" = 4 ]; then
            echo
	    echo "Enter the name of a file to edit"
	    read filenameX
	    if [ ! -e $filenameX ]; then
		echo "Couldn't find $filenameX!"
	    else
		"$EDITOR" $filenameX &
	    fi
        elif [ "$get_num" = 5 ]; then
	    echo "idf.py fullclean"
	    idf.py fullclean
        elif [ "$get_num" = 6 ]; then
            echo "Building C++..."
	    idf.py build
        elif [ "$get_num" = 7 ]; then
            echo "Compiling DSP code..."
            faust2esp32 -lib -ac101 main/$filename.dsp
            if [ $? -ne 0 ]; then
	        echo "Faust compiler error!"
	        exit 1 
            fi
            unzip $filename.zip
            if [ $? -ne 0 ]; then
            	echo "unzip error!"
	        exit 1 
            fi
            mv $filename/*.* main
        elif [ "$get_num" = 8 ]; then
            echo "Building, flashing, monitor..."
	    idf.py -p $PORT -b $BAUD flash monitor
        elif [ "$get_num" = 9 ]; then
            echo "Monitor..."
	    idf.py -p $PORT -b $BAUD monitor
        elif [ "$get_num" = 10 ]; then
            echo "Make menuconfig..."
	    make menuconfig
        elif [ "$get_num" = 11 ]; then
            echo "Saving reference..."
	    savePath=../reference/$filename
	    if [ ! -d "$savePath" ]; then
		    mkdir "$savePath"
	    fi
	    echo
	    echo "Enter name of tar file to save:"
	    read tarfilename
	    if [ ! -e "$savePath"/$tarfilename.tar ]; then
                cp main/main.cpp "$savePath"
	        cp main/$filename.cpp "$savePath"
	        cp main/$filename.dsp "$savePath"
	        cp build/$filename.elf "$savePath"
	        cp sdkconfig "$savePath"
		pushd "$savePath"
		tar cvf $tarfilename.tar main.cpp $filename.cpp $filename.dsp sdkconfig $filename.elf
		rm $tarfilename.tar main.cpp $filename.cpp $filename.dsp sdkconfig $filename.elf
		popd
	    else
		echo "Target file $savePath/$tarfileanme already exists!"
		echo "Press a key to return to the menu."
		echo
		read anykey
 	    fi
        elif [ "$get_num" = 12 ]; then
           if ask "Are you sure you want to exit?"; then
                exit 0
            fi
        fi
    done
}

############ End menuLoop #######################

filename=$1
menuLoop
