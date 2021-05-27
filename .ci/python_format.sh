#!/bin/bash
IGNORED_PATHS="aleph2_gui/src/aleph2_gui/(resources)"
FORMAT_COMMAND_BASE="black . --extend-exclude"
$FORMAT_COMMAND_BASE $IGNORED_PATHS --check
ret=$?
if [ $ret -ne 0 ]
then
	echo "Some files were not correctly formatted"
	echo "You may want to run \"$FORMAT_COMMAND_BASE '$IGNORED_PATHS'\""
fi
exit $ret
