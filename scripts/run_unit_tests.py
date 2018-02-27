##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM
import time
import glob
import sys
import os
import _thread as thread

import subprocess as sub

TEST_PATH = '/TapGameController/tests/'
TEST_EXEC_PREFIX = 'TapGameController.tests.'


def main():

    #get list of all test files
    print(os.getcwd())
    test_files = glob.glob(os.getcwd() + TEST_PATH + 'test_*.py')

    for i in range(0, len(test_files)):
        test_files[i] = test_files[i].replace('.py', '')
        test_files[i] = test_files[i].strip()
        test_files[i] = test_files[i].split('/')[-1]

    print("my files")
    print(test_files)

    for i in range(0, len(test_files)):
        try:
            print(['python', '-W', 'ignore', '-m', TEST_EXEC_PREFIX + test_files[i]])
            p = sub.run(['python', '-m', TEST_EXEC_PREFIX + test_files[i]], stdout=sub.PIPE, stderr=sub.PIPE, encoding='utf-8')
            print(p.stdout)
            print(p.stderr)

        except :
            #print(e)
            sys.exit()

main()