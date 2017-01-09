  python3 robot/robot.py test

  num_errors=`find ./robot -name \*.py -exec pep8 --ignore=E402,E501 {} + | wc -l`
  if [ "$num_errors" -gt 0 ]; then find ./robot -name \*.py -exec pep8 --ignore=E402,E501 {} +; exit 1; fi

  find ./robot -name \*.py -exec python3 tests/test_print.py {} \;
