# -*- coding: utf-8 -*-
import torch

print("GPU是否可用:", torch.cuda.is_available())
print("GPU可用数量:", torch.cuda.device_count())
