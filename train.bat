cd sample-env\Scripts
call activate.bat
cd ../../RL_train
mlagents-learn NICO.yaml --run-id=test8
pause