trap "exit 1" TERM
export TOP_PID=$$
python3 robot/robot.py test
#num_errors=`find ./robot -name \*.py -exec pep8 --ignore=E402,E501 {} + | wc -l`
#if [ "$num_errors" -gt 0 ]; then find ./robot -name \*.py -exec pep8 --ignore=E402,E501 {} +; exit 1; fi



print () {
python3 tests/test_print.py $1;
rc=$?; if [[ $rc != 0 ]]; then kill -s TERM $TOP_PID; fi;
}
export -f print

#find ./robot -name \*.py -exec bash -c 'print "$0"' {} \;
