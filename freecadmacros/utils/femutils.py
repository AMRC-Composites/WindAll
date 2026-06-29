"""
Useful utilities for dealing with fem and fem results, 
intended to be run from within FreeCAD
"""

import FreeCAD


def find_max_stress(stress_name, vtk_obj, print_res=False):
    """
    Function to find the location of maximum stress (or strain or displacement)
    Requires:
    stress_name: string of the name of field to find max of
    vtk_obj: the FreeCAD result object containing the VTK data
    print_res: Bool to print results to the console or not
    Returns:
    max_stress: the highest value of the field
    max_stress_pid: id of point of maximum value
    max_stress_loc: FreeCAD Vector for the location of the maximum
    """

    data = vtk_obj.Data
    pdata = data.GetPointData()
    if not pdata.GetArray(stress_name):
        FreeCAD.Console.PrintWarning(stress_name, "NOT FOUND IN DATA SET")
        return None, None, None
    max_stress = 0  # Maximum value of stress component
    max_stress_pid = None  # id of point of maximum stress
    max_stress_loc = None  # location of maximum stress

    for i in range(data.GetNumberOfPoints()):
        stress = pdata.GetArray(stress_name).GetValue(i)
        if abs(stress) > abs(max_stress):
            max_stress = stress
            max_stress_pid = i
            max_stress_loc = FreeCAD.Vector(data.GetPoint(i)[0],
                                            data.GetPoint(i)[1],
                                            data.GetPoint(i)[2])

    if max_stress:
        if print_res:
            FreeCAD.Console.PrintMessage(stress_name + "\n")
            FreeCAD.Console.PrintMessage(f"Maximum of: {max_stress}\n")
            FreeCAD.Console.PrintMessage(f"On point id: {max_stress_pid}\n")
            FreeCAD.Console.PrintMessage(f"At location: {max_stress_loc}\n")
    else:
        FreeCAD.Console.PrintWarning(f"Maximum of {stress_name} NOT FOUND")

    stress_name_axis = stress_name + "_axis"
    axisdata = pdata.GetArray(stress_name_axis)
    if axisdata:
        xval = axisdata.GetComponent(0, 0)
        yval = axisdata.GetComponent(0, 1)
        axis = FreeCAD.Vector(xval, yval, 0)
    else:
        axis = None

    return max_stress, max_stress_pid, max_stress_loc, axis
