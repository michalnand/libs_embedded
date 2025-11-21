import numpy
import torch

class Smoothing(torch.nn.Module):
    def __init__(self, kernel_size = 7):
        super(Smoothing, self).__init__()
        self.filter = torch.nn.AvgPool2d(kernel_size, stride = 1, padding=kernel_size//2)

    def forward(self, x):
        return self.filter(x)
        

class AdaptiveThresholding(torch.nn.Module):
    def __init__(self, kernel_size = 7):
        super(AdaptiveThresholding, self).__init__()

        self.filter = torch.nn.AvgPool2d(kernel_size, stride = 1, padding=kernel_size//2)

    def forward(self, x, threshold = 0.01):
        d  = x - self.filter(x)

        y = (d > threshold).float()
        y = y[0].detach().to("cpu").numpy()

        return y


class FeatuesPoints(torch.nn.Module):
    def __init__(self, threshold = 0.5):
        super(FeatuesPoints, self).__init__()

        #laplacian of Gaussian filter
        kernel = [
            [0, 0, 1,  1, 1, 0, 0],
            [0, 1, 3,  3, 3, 1, 0],
            [1, 3, 0, -7, 0, 3, 0],
            [1, 3,-7, -24,-7,3, 1],
            [1, 3, 0, -7, 0, 3, 0],
            [0, 1, 3,  3, 3, 1, 0],
            [0, 0, 1,  1, 1, 0, 0]
        ]

        kernel = torch.tensor(kernel)
        kernel = kernel/torch.max(torch.abs(kernel))

        self.conv   = torch.nn.Conv2d(1, 1, kernel_size=7, stride=1, padding=7//2, bias=False)
        self.conv.weight.data[0][0] = kernel

        self.max_pool   = torch.nn.MaxPool2d(3, stride=1, padding=1)
        self.erosion    = torch.nn.AvgPool2d(3, stride=1, padding=3//2)
        
        self.threshold   = threshold
        
    def forward(self, x):
        y = x.clone()

        for i in range(4):
            y = self.conv(y)
            y = self.max_pool(y)            
        
        y = 1.0*(y > self.threshold)

        y = self.erosion(y)

        y = 1.0*(y > 0.8)

        indices = torch.where(y > 0.5)

        return y, indices


