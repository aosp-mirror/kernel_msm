#!/bin/bash

#
# Colorcodes
#
NORMAL=`echo -e '\033[0m'`
RED=`echo -e '\033[31m'`
GREEN=`echo -e '\033[0;32m'`
LGREEN=`echo -e '\033[1;32m'`
BLUE=`echo -e '\033[0;34m'`
LBLUE=`echo -e '\033[1;34m'`
YELLOW=`echo -e '\033[0;33m'`

unset MENU_CHOICES
WORKSPACE=$PWD

function add_menu_combo()
{
    local new_combo=$1
    local c
    for c in ${MENU_CHOICES[@]} ; do
        if [ "$new_combo" = "$c" ] ; then
            return
        fi
    done
    MENU_CHOICES=(${MENU_CHOICES[@]} $new_combo)
}

# add the default one here
add_menu_combo update_source_code
add_menu_combo show_git_log
add_menu_combo Quit

# ======================================================================================================================

function print_menu()
{
    local uname=$(uname)
    echo "Lunch menu... pick a combo:"

    local i=1
    local choice
    for choice in ${MENU_CHOICES[@]}
    do
        echo -e "     $i. $choice"
        i=$(($i+1))
    done
}

function select_menu()
{
    local answer
    print_menu
    read answer

    selection_menu=

    if [ -z "$answer" ]
    then
        selection_menu=update_source_code
    elif (echo -n $answer | grep -q -e "^[0-9][0-9]*$")
    then
        if [ $answer -le ${#MENU_CHOICES[@]} ]
        then
            selection_menu=${MENU_CHOICES[$(($answer-1))]}
        fi
    else
        selection_menu=$answer
    fi

}

while [ "$selection_menu" != "Quit" ]
do
	select_menu

	if [ "$selection_menu" == "update_source_code" ]
	then
		echo "Update source code..."
	fi
	
	if [ "$selection_menu" == "show_git_log" ]
	then
		./gshow.sh
	fi

done

echo ${NORMAL}
