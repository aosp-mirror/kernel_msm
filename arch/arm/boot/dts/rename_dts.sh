#!/bin/bash
PROJECT_CODE="sardine"
NEW_PROJECT_CODE="ling"
DTS_FOLDER="$1"
#FILES=$(grep $PROJECT_CODE $DTS_FOLDER/ * -r)
FILES=$(grep $PROJECT_CODE * -r )
for file in $FILES
do
	dts_file=$(echo $file | sed -n 's/\(.*\):\(.*\)/\1/p')
	if [ "$dts_file" != "" ]; then
		echo "$PWD/$dts_file"
		sed  -i "s/sardine/ling/g" $PWD/$dts_file
		echo "done"
	fi
done

OLD_FILES=$(find . -name '*sardine*')
for file in $OLD_FILES
do
#	echo $file
	if [ "$file" != "" ]; then
		#echo $file
		new_file=$(echo "$file" | sed -e 's/sardine/ling/g')
		echo $new_file
		mv $file $new_file
	fi
done
