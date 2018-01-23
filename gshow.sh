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

unset PROJECT_MENU_CHOICES
unset MENU_ITEM_CHOICES
unset PROJECT_ITEM
WORKSPACE=$PWD

function add_project_combo()
{
    local new_combo=$1
    local c
    for c in ${PROJECT_MENU_CHOICES[@]} ; do
        if [ "$new_combo" = "$c" ] ; then
            return
        fi
    done
    PROJECT_MENU_CHOICES=(${PROJECT_MENU_CHOICES[@]} $new_combo)
}

# add the default one here
add_project_combo sculpin
add_project_combo lionfish
add_project_combo bluegill
add_project_combo shiner
add_project_combo sundial
add_project_combo stargazer
add_project_combo glowlight
add_project_combo all

function add_menu_combo()
{
    local new_combo=$1
    local c
    for c in ${MENU_ITEM_CHOICES[@]} ; do
        if [ "$new_combo" = "$c" ] ; then
            return
        fi
    done
    MENU_ITEM_CHOICES=(${MENU_ITEM_CHOICES[@]} $new_combo)
}

# add the default one here
add_menu_combo bootloader
add_menu_combo prebuilt_kernel
add_menu_combo kernel

# ======================================================================================================================

function print_project_menu()
{
    local uname=$(uname)
    echo "Lunch menu... pick a combo:"

    local i=1
    local choice
    for choice in ${PROJECT_MENU_CHOICES[@]}
    do
        echo -e "     $i. $choice"
        i=$(($i+1))
    done
}

function select_project()
{
    local answer
    if [ "$1" ] ; then
        answer=$1
    else
        print_project_menu
        echo -n "Which would you like? [sculpin]"
        read answer
    fi

    selection_project=

    if [ -z "$answer" ]
    then
        selection_project=sculpin
    elif (echo -n $answer | grep -q -e "^[0-9][0-9]*$")
    then
        if [ $answer -le ${#PROJECT_MENU_CHOICES[@]} ]
        then
            selection_project=${PROJECT_MENU_CHOICES[$(($answer-1))]}
        fi
    else
        selection_project=$answer
    fi

}
# ======================================================================================================================

function print_show_item_menu()
{
    local uname=$(uname)
    echo "Lunch menu... pick a combo:"

    local i=1
    local choice
    for choice in ${MENU_ITEM_CHOICES[@]}
    do
        echo -e "     $i. $choice"
        i=$(($i+1))
    done
}

function select_show_item()
{
    local answer
    if [ "$1" ] ; then
        answer=$1
    else
		print_show_item_menu
        echo -n "Which would you like? [bootloader]"
        read answer
    fi

    selection_show_item=

    if [ -z "$answer" ]
    then
        selection_show_item=bootloader
    elif (echo -n $answer | grep -q -e "^[0-9][0-9]*$")
    then
        if [ $answer -le ${#MENU_ITEM_CHOICES[@]} ]
        then
            selection_show_item=${MENU_ITEM_CHOICES[$(($answer-1))]}
        fi
    else
        selection_show_item=$answer
    fi

}

echo "Show Status..."
echo

select_project $1
#print_project_menu $1

select_show_item $2

function show_bootloader_status()
{
	local lproject=$1
	echo "${GREEN}Show the latest $lproject bootloader verison${NORMAL}"
	cd $WORKSPACE/gvendor/compal/$lproject
	#git pull
	git log --oneline | grep bootloader --color | sed -n '1p'
}

function show_prebuilt_kernel_status()
{
    local lproject=$1
	echo "${GREEN}Show the latest device/compal/$lproject-kernel${NORMAL}"
	cd $WORKSPACE/gdevice/$lproject/$lproject-kernel
	#git pull
	git show | sed -n '7p'
}

function show_kernel_status()
{
	local lproject=$1
	echo "${GREEN}Show the latest kernel log${NORMAL}"
	git log --oneline -1 android-msm-$lproject-3.18
}

function get_current_project_status()
{
    local new_combo=$1
    local c
    for c in ${PROJECT_MENU_CHOICES[@]} ; do
		if [ "$c" == "all" ] 
		then
			return
		fi

		if [ "$selection_show_item" == "bootloader" ]
		then
			show_bootloader_status $c
		fi

		if [ "$selection_show_item" == "prebuilt_kernel" ]
		then
			show_prebuilt_kernel_status $c
		fi

		if [ "$selection_show_item" == "kernel" ]
		then
			show_kernel_status $c
		fi

		echo	
        if [ "$new_combo" = "$c" ] ; then
            return
        fi
    done                                        
#    PROJECT_ITEM=(${PROJECT_ITEM[@]} $new_combo)
}

if [ "$selection_project" == "all" ]
then
	get_current_project_status 
exit

fi

if [ "$selection_show_item" == "prebuilt_kernel" ]
then
	prebuilt_kernel_status $selection_project
if

if [ "$selection_show_item" == "bootloader" ]
then
	show_bootloader_status $selection_project
fi

echo ${NORMAL}
