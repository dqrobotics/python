pipeline {
  agent any
  stages {
    stage('error') {
      steps {
        sh '''python3 -m venv venv
source venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install wheel setuptools setuptools-git

cd tmp
cd python
git checkout master
python setup.py bdist_wheel
rm -rf build
python -m pip install dist/*.whl

cd tests
python -m pip install scipy quadprog
python DQ_test.py
python DQ_Kinematics_test.py
python cpp_issues.py
python python_issues.py
cd .. #tests


cd .. #python'''
      }
    }

  }
}