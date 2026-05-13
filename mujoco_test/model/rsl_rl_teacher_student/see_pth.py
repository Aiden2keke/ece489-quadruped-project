import torch

# 替换为你的.pth文件路径
pth_file = '/home/yd/program/rsl_rl_teacher_student/mujoco_test/model/rsl_rl_teacher_student/actor/actor_oracle_TS_DR_2.pth'

# 加载.pth文件，通常返回一个字典，包含模型权重、优化器状态等
model_data = torch.load(pth_file, map_location='cpu')  # map_location='cpu'避免GPU依赖

# 查看字典中的所有键
print("Keys in the pth file:", model_data.keys())

# 如果你想查看模型权重（通常在 'state_dict' 或 'model' 键中）
if 'state_dict' in model_data:
    print("Model state_dict keys:", model_data['state_dict'].keys())
elif 'model' in model_data:
    print("Model keys:", model_data['model'].keys())
else:
    # 如果是直接保存的权重字典
    print("Model keys:", model_data.keys())

# 你也可以打印部分权重张量的形状，了解模型结构
for key, value in model_data.get('state_dict', model_data).items():
    print(f"{key}: {value.shape if hasattr(value, 'shape') else type(value)}")
