#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version = '1.0'
# ---------------------------------------------------------------------------
"""Test file for intersection class implementation"""
# ---------------------------------------------------------------------------
# Imports
from crossing import Crossing
from intersection import intersection
import pandas as pd

# feature_df = pd.read_excel('./INT10_BEFORE_AM_Bathurst-Carrville.xlsx',sheet_name="SummaryInput")
# feature_df = pd.read_excel('./INT11_BEFORE_AM_Bathurst-Clark.xlsx',sheet_name="SummaryInput")
feature_df = pd.read_excel('./INT12_BEFORE_AM_Mackenzie-Bayview.xlsx',sheet_name="SummaryInput")

intersection_test = intersection(feature_df)

for ind, cross in enumerate([intersection_test.crossing1,
                             intersection_test.crossing2,
                             intersection_test.crossing3,
                             intersection_test.crossing4]):
    print(100*"=")
    print("Crossing " + str(ind+1))
    print("PCV values: ", cross.PCV)
    print("PPP values: ", cross.PPP)
    print("CS values: ", cross.CS)
    print("PSI injury values: ", cross.PSI_injury)
    print("PSI fatal values: ", cross.PSI_death)
    


print("*"*100)
print("Intersection:")
print("PCV values: ", intersection_test.PCV)
print("PSI death risk values: ", round(sum(intersection_test.PSI_death),3))
print("PSI severe injury risk values: ",  round(sum(intersection_test.PSI_injury),3))

