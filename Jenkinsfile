pipeline {
  agent any
  stages {
    stage('Setting PATH') {
      steps {
        sh '''# Set environment to use homebrew
PATH=$PATH:/opt/homebrew/bin

'''
      }
    }

    stage('Configure Python VENV') {
      steps {
        sh '''python3 -m venv venv
source venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install wheel setuptools setuptools-git
'''
      }
    }

    stage('Build') {
      steps {
        sh '''python setup.py bdist_wheel
rm -rf build
python -m pip install dist/*.whl
'''
      }
    }

    stage('Test') {
      steps {
        sh '''cd tests
python -m pip install scipy quadprog
python DQ_test.py
python DQ_Kinematics_test.py
python cpp_issues.py
python python_issues.py
cd .. #tests'''
      }
    }

    stage('') {
      steps {
        cleanWs(cleanWhenAborted: true, cleanWhenFailure: true, cleanWhenNotBuilt: true, cleanWhenSuccess: true, cleanWhenUnstable: true, cleanupMatrixParent: true, deleteDirs: true)
      }
    }

  }
}