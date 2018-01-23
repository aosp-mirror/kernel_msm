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

echo "Update the code from Google server..."
echo

select_project $1
print_project_menu $1

echo
echo "${GREEN}Update device/compal/$selection_project"
echo ${NORMAL}
cd $WORKSPACE/gdevice/$selection_project/$selection_project
git pull


echo
echo "${GREEN}Update device/compal/$selection_project-kernel"
echo ${NORMAL}
cd $WORKSPACE/gdevice/$selection_project/$selection_project-kernel
git pull


echo
echo "${GREEN}Update vendor/compal/$selection_project"
echo ${NORMAL}
cd $WORKSPACE/gvendor/compal/$selection_project
git pull


echo
echo "${GREEN}Update vendor/pdk/$selection_project/$selection_project-userdebug/platform"
echo ${NORMAL}
cd $WORKSPACE/gvendor/pdk/$selection_project/$selection_project-userdebug/platform
git fetch
git rebase
git rebase --skip


echo "${GREEN}All update done!"
echo ${NORMAL}
