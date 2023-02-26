import numpy


class FilterNode:

    def __init__(self, filter_type:str, center_frequency = 0.0, q_factor = 1.0, f_sampling = 1.0, prev = None):

        self.filter_type        = filter_type
        self.center_frequency   = center_frequency
        self.q_factor           = q_factor
        self.f_sampling         = f_sampling

        if prev is not None:
            self.prev_list      = prev.prev_list
        else:
            self.prev_list      = []
        self.others             = []

        self.prev_list.append(self)

    def append(self, other):
        self.others.append(other)

    
    def traverse(self, stage = 0):

       
        '''
        for i in range(len(self.prev_list)):
            if len(self.others) > 0:
                stage_ = stage+1
            else:
                stage_ = stage

            s, filter_call_func = self.prev_list[i].get(stage_, i, i+1)
            tmp = int(stage > 0)
            print("\t"*(i + tmp), end="")
            print(s)
            #print(filter_call_func, s)
        '''
        
        for i in range(len(self.others)):
            self.others[i].traverse(i + 1)
            print("\n")
       
        '''
        #max_depth = self._get_depth() - 1

        for i in range(stage):
            print("\t", end="")
        print(stage, self.filter_type, self.center_frequency, "Hz")

        if self.parent is not None:
            self.parent.traverse(stage+1)
        
        for i in range(len(self.others)):
            self.others[i].traverse(stage+1)
            print("\n")
        '''
    
    def get(self, stage, input, output):
        s = " //"
        s+= str(self.filter_type) + "_" + str(stage) + "_" + str(output) + "\t"
        s+= str(self.center_frequency) + "Hz " + "\t"
        s+= "Q=" + str(self.q_factor)
        
        filter_call_func = "x_" + str(stage) + "_" + str(output) + " = "
        filter_call_func+= "filter" + "_" + str(stage) + "_" + str(output) + ".forward("
        filter_call_func+= "x_" + str(stage) + "_" + str(input) + ");"

        return s, filter_call_func


    def export_calls(self, curr_stage = 0):
        max_depth = self._get_depth() - 1

        
        stage = int(len(self.others))
        if self.parent is not None:
            self.parent.export_calls(max_depth)
        
        for i in range(max_depth ):
            print("\t", end="")
        print(curr_stage, self.filter_type, self.center_frequency, "Hz")

        
        for i in range(len(self.others)):
            self.others[i].export_calls(curr_stage)
            print("\n")
        
        

    def _get_depth(self, depth_curr = 0):
        if self.parent is not None:
            depth_curr = self.parent._get_depth(depth_curr)
        
        return depth_curr + 1
        

