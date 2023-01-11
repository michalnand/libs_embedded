import numpy
import torch

import matplotlib.pyplot as plt


def create_transform(tx, ty, sx, sy, skew, angle):

    #translation
    wt = numpy.eye(3)
    wt[0][2] = tx
    wt[1][2] = ty

    #scaling
    st = numpy.eye(3)
    st[0][0] = sx
    st[1][1] = sy

    #skew
    sct = numpy.eye(3)
    sct[0][1] = skew
    
    #rot
    rot = numpy.eye(3) 
    rot[0][0] = numpy.cos(angle)
    rot[0][1] = -numpy.sin(angle)
    rot[1][0] = numpy.sin(angle)
    rot[1][1] = numpy.cos(angle)

    w = wt@st@sct@rot

    return w

def create_transform_t(tx, ty, sx, sy, skew, angle):

    #translation
    wt = torch.eye(3)
    wt[0][2] = tx
    wt[1][2] = ty

    #scaling
    st = torch.eye(3)
    st[0][0] = sx
    st[1][1] = sy

    #skew
    sct = torch.eye(3)
    sct[0][1] = skew
    
    #rot
    rot = torch.eye(3)
    rot[0][0] = torch.cos(angle)
    rot[0][1] = -torch.sin(angle)
    rot[1][0] = torch.sin(angle)
    rot[1][1] = torch.cos(angle)

    w = wt@st@sct@rot

    return w

def apply_transform(points, w):

    ones        = numpy.ones((points_a.shape[0], 1))

    points_tmp  = numpy.concatenate([points, ones], axis=1)


    result = numpy.matmul(points_tmp, w.T)

    return result[:, 0:2]


def normalise(x):

    min_v = torch.min(x, dim=0)[0]
    max_v = torch.max(x, dim=0)[0]

    k = 2.0/(max_v - min_v)
    q = 1.0 - k*max_v

    x_norm = k*x + q

    return x_norm
   

def find_transform(points_a, points_b, steps = 200, lr=0.5):

    pa_t = torch.from_numpy(points_a.copy()).float()
    pb_t = torch.from_numpy(points_b.copy()).float()

    
    #expand ones column
    ones_t  = torch.ones((points_a.shape[0], 1)).float()
    pa_t    = torch.cat([pa_t, ones_t], dim=1)
    pb_t    = torch.cat([pb_t, ones_t], dim=1)    

    #initial parameters
    f_initial = torch.eye(3)
    f_mat     = torch.nn.Parameter(f_initial, requires_grad = True)

    optimizer = torch.optim.Adam([f_mat], lr=lr)

    
    for i in range(steps):
        p = torch.matmul(pa_t, f_mat.T)
        loss_f = ((pb_t - p)**2).mean()

        loss_norm = (1.0 - (f_mat**2).sum())**2

        loss = loss_f #+ loss_norm

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return f_mat.detach().cpu().numpy()



def find_transform_elements(points_a, points_b, steps = 200, lr=0.5):

    pa_t = torch.from_numpy(points_a.copy()).float()
    pb_t = torch.from_numpy(points_b.copy()).float()

    
    #expand ones column
    ones_t  = torch.ones((points_a.shape[0], 1)).float()
    pa_t    = torch.cat([pa_t, ones_t], dim=1)
    pb_t    = torch.cat([pb_t, ones_t], dim=1)    

    #initial parameters
    tx     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    ty     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    sx     = torch.nn.Parameter(torch.ones((1, )).float(), requires_grad = True)
    sy     = torch.nn.Parameter(torch.ones((1, )).float(), requires_grad = True)
    skew     = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)
    angle  = torch.nn.Parameter(torch.zeros((1, )).float(), requires_grad = True)

    optimizer = torch.optim.Adam([tx, ty, sx, sy, skew, angle], lr=lr)

    
    for i in range(steps):

        f_mat = create_transform_t(tx, ty, sx, sy, skew, angle)

        p = torch.matmul(pa_t, f_mat.T)
        loss = ((pb_t - p)**2).mean()


        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    

    tx      = tx.detach().cpu().numpy()[0]
    ty      = ty.detach().cpu().numpy()[0]
    sx      = sx.detach().cpu().numpy()[0]
    sy      = sy.detach().cpu().numpy()[0]
    skew    = skew.detach().cpu().numpy()[0]
    angle   = angle.detach().cpu().numpy()[0]

    return tx, ty, sx, sy, skew, angle



if __name__ == "__main__":
    points_count    = 256
    
    points_a    = 100.0*numpy.random.randn(points_count, 2) + 100.0*numpy.random.randn(1, 2)


    tx      = 10.0*numpy.random.randn()
    ty      = 10.0*numpy.random.randn()
    sx      = 2.0*numpy.random.rand()
    sy      = 2.0*numpy.random.rand()
    skew    = numpy.random.rand()
    rot     = numpy.pi*(2.0*numpy.random.rand() - 1.0)
 
    print("translation = ", round(tx, 4), round(ty, 4))
    print("scaling     = ", round(sx, 4), round(sy, 4))
    print("skew        = ", round(skew, 4))
    print("rotation    = ", round(rot, 4))

    w_ref = create_transform(tx, ty, sx, sy, skew, rot)

    points_b    = apply_transform(points_a, w_ref)

    

    tx, ty, sx, sy, skew, rot = find_transform_elements(points_a, points_b)
    w_test = create_transform(tx, ty, sx, sy, skew, rot)
    points_test = apply_transform(points_a, w_test)

    print("\n\n")
    print("computed")
    print("translation = ", round(tx, 4), round(ty, 4))
    print("scaling     = ", round(sx, 4), round(sy, 4))
    print("skew        = ", round(skew, 4))
    print("rotation    = ", round(rot, 4))


    
    plt.scatter(points_a[:, 0], points_a[:, 1], label="points_a")
    plt.scatter(points_b[:, 0], points_b[:, 1], label="points_b")
    plt.scatter(points_test[:, 0], points_test[:, 1], s=5, label="points_b computed")
    plt.legend()
    plt.show()