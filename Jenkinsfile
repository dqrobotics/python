pipeline {
  agent any
  stages {
    stage('Build') {
      steps {
        sh '''# Set environment to use homebrew
PATH=$PATH:/opt/homebrew/bin

# Set up build enviroment
python3 -m venv venv
source venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install wheel setuptools setuptools-git

# Build
python setup.py bdist_wheel
rm -rf build
python -m pip install dist/*.whl
'''
      }
    }

    stage('Test') {
      steps {
        sh '''# Set up test environment
source venv/bin/activate


# Run tests
cd tests
python -m pip install scipy quadprog
python DQ_test.py
python DQ_Kinematics_test.py
python cpp_issues.py
python python_issues.py
cd .. #tests'''
      }
    }

    stage('Deploy') {
      steps {
        sh 'echo $BRANCH_NAME'
      }
    }

    stage('Delete Workspace') {
      steps {
        cleanWs(cleanWhenAborted: true, cleanWhenFailure: true, cleanWhenNotBuilt: true, cleanWhenSuccess: true, cleanWhenUnstable: true, cleanupMatrixParent: true, deleteDirs: true)
        cleanWs(cleanWhenAborted: true, cleanWhenFailure: true, cleanWhenNotBuilt: true, cleanWhenSuccess: true, cleanWhenUnstable: true, cleanupMatrixParent: true, deleteDirs: true)
      }
    }

  }
}