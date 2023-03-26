import pathlib
import numpy as np

class DebugParser:
    def __init__(self, path) -> None:
        self.path = pathlib.Path(path)
        self.data_dict = {}

    def parse(self):
        with open(self.path.resolve()) as f:
            lines = f.readlines()
        #Find line with "Terminal ready", after that all debug data should come
        for i in range(len(lines)):
            if lines[i] == 'Terminal ready\n':
                debugdata = lines[i+1:]

        #Check that two consequtive lines have the begining, to avoid malformated lines
        for j in range(len(debugdata)):        
            if (debugdata[j][0:2] == debugdata[j+1][0:2]):
                    debugdata = debugdata[j:]
                    break

        #For all the lines in the debugdata, find identifier and value
        for line in debugdata:
            line = line.strip()
            line = line.strip('\x00')
            identifier, value, rest = self.parse_identifier_value(line)

            if rest == None:
                break
            
            identifier = self.clean_idenetifier(identifier)
            
            if identifier not in self.data_dict:
                self.data_dict[identifier] = [value]
            else:
                self.data_dict[identifier].append(value)
            while rest != None:

                identifier, value, rest = self.parse_identifier_value(rest)
                
                if rest == None:
                    break
                else:
                    identifier = self.clean_idenetifier(identifier)
                    if identifier not in self.data_dict:
                        self.data_dict[identifier] = [value]
                    else:
                        self.data_dict[identifier].append(value)

    def clean_idenetifier(self, identifier) -> str:
        if identifier != None:
            i = 0
            for char in identifier:
                i += 1
                #In the identifier the unit from previous value are included. After the space is acutal identifier
                if char.isspace():
                    identifier = identifier[i:]
                    return identifier
            return identifier

    def parse_identifier_value(self, line) -> tuple([str, int, str]):
        #parse for an indetifier, after the identifier an "=" should come, after that the value should come
        #Eg, "qsz = 3 XXXXXXXX" becomes 
        #identifier = qsz
        #Value = 3
        #Rest = XXXXXXX
        for char_i in range(len(line)):
            char = line[char_i]
            if char == '=':
                identifier = line[:char_i].strip()
                rest = line[char_i+1:].strip()
                value, rest = self.parse_value(rest)
                #print("Identifier:", identifier, " Value:", value)
                return (identifier, value, rest)
        #If nothing is found, return None for all
        return (None, None, None)

    def parse_value(self, line) -> tuple([int,list]):
        #Convert the digits to an integer, return the value and rest of the string.
        line = line.strip()
        i = 0
        isFloat = False
        #print("Len: ",len(line), " Line =", line, "End")
        #Only loop if len is greater than 1
        if len(line) > 1:
            while (line[i].isnumeric() or (line[i] == '-') or (line[i] == '.')):
                if (line[i] == '.'):
                    isFloat = True
                i += 1
                if (i-1 == len(line)-1):
                    break

            if isFloat:
                value = float(line[:i])
            else:
                value = int(line[:i])
        else:
            if isFloat:
                value = float(line)
            else:
                value = int(line)
        return (value, line[i+1:])
    def get_dts(self):
        dts = []
        for i in range(len(self.data_dict["t"])-1):
            dt = self.data_dict["t"][i+1] - self.data_dict["t"][i] 
            dts.append(dt)

        return dts
    
    def get_state_dict(self):
        state_dict = {"top_angle": np.array(list(map(float,self.data_dict["theta"])))/1000,
                      "top_angle_d": np.array(list(map(float,self.data_dict["theta_d"])))/1000,
                      "wheel_position": np.array(list(map(float,self.data_dict["x"]))),
                      "wheel_position_d": np.array(list(map(float,self.data_dict["x_d"]))),
                      "dt": np.array(list(map(float,self.get_dts())))/1000}  
        return state_dict    

    

