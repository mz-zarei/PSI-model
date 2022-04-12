#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version = '1.0'
# ---------------------------------------------------------------------------
"""Computing PSI values for all given intersections"""
# ---------------------------------------------------------------------------
# Imports
from curses.ascii import SI
# from crossing_v1 import Crossing
from crossing_v3 import Crossing

from intersection import intersection
import pandas as pd
import os

inputsPath = './Inputs'

    
# Output lists for final dataframe
IDs = []
Crosswalks = []
ConflictZones = []
Movements = []
PCVs = []
PPPs = []
CSs = []
DRs = []
SIRs = []


with open('Outputs/out.txt', 'w') as f:
    for path in os.listdir(inputsPath):
        f.write("%"*90 + "\n")
        f.write("%"*90 + "\n")
        f.write("Crossings for           " + path[2:-5] + "\n")
        f.write("%"*90 + "\n")
        f.write("%"*90 + "\n")

        feature_df = pd.read_excel(os.path.join(inputsPath, path),sheet_name="SummaryInput")

        intersection_test = intersection(feature_df)

        for cross, dir in zip([intersection_test.crossing1, intersection_test.crossing2, intersection_test.crossing3, intersection_test.crossing4],
                               ['North', 'West', 'South', 'East']):
            f.write(90*"=" + "\n")
            f.write(f"Crossing:          {dir} \n")
            f.write(f"PCV values:        {cross.PCV} \n")
            f.write(f"PPP values:        {cross.PPP} \n")
            f.write(f"CS values:         {cross.CS} \n")
            f.write(f"PSI injury values: {cross.PSI_injury} \n")
            f.write(f"PSI fatal values:  {cross.PSI_death} \n")

            IDs.extend([path]*5)
            Crosswalks.extend([dir]*5)
            ConflictZones.extend(['A', 'C', 'B', 'D', 'A'])
            Movements.extend(['RT1', 'RT1', 'RT2', 'RT2', 'LT3'])
            PCVs.extend(cross.PCV)
            PPPs.extend(cross.PPP)
            CSs.extend(cross.CS)
            DRs.extend(cross.DR)
            SIRs.extend(cross.SIR)

        f.write("*"*90 + "\n")
        f.write("Final results for intersection:                   " + path[2:-5] + "\n")
        f.write(f"PCV values for each crossing:                    {intersection_test.PCV} \n")
        f.write(f"PSI death risk values for each crossing:         {intersection_test.PSI_death} \n")
        f.write(f"PSI severe injury risk values for each crossing: {intersection_test.PSI_injury} \n\n\n")

outputDF = pd.DataFrame()
outputDF['IDs'] = IDs
outputDF['Crosswalks'] = Crosswalks
outputDF['ConflictZones'] = ConflictZones
outputDF['Movements'] = Movements
outputDF['Potential Conflict Volume'] = PCVs
outputDF['Ped Presence Prob'] = PPPs
outputDF['Conflict Speed'] = CSs
outputDF['Death Risk'] = DRs
outputDF['Injury Risk'] = SIRs

outputDF.to_excel('./Outputs/out.xlsx')