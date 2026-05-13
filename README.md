### Usage ###
1. Teacher-Student Train:  
  ```python legged_gym/scripts/train.py --task=go2 --headless```
1. Student Reinforcing Train:  
  ```python legged_gym/scripts/train.py  --task=go2 --headless --max_iterations=1500 --student_reinforcing --resume --experiment_name=rough_go2 --run_name=TS_re3 --checkpoint=7500```
1. Play:
  ```python legged_gym/scripts/play.py --task=go2 --load_run=TS_re3 --checkpoint=9000```
