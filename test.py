#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version = '1.0'
# ---------------------------------------------------------------------------
"""Test file for Crossing implementation"""
# ---------------------------------------------------------------------------
# Imports
from curses.ascii import SI
# from crossing_v1 import Crossing
from crossing_v3 import Crossing

from intersection import intersection
import pandas as pd

path = "/Users/mz/Documents/GitHub_Projects/PSI_model/Inputs/Conflict Frequency Calculator V3e.xlsx"
feature_df = pd.read_excel(path,sheet_name="SummaryInput")

intersection_test = intersection(feature_df)
print(['A',     'C',   'B',   'D', 'A'])
print(['RT1', 'RT1', 'RT2', 'RT2', 'LT3'])
print(intersection_test.crossing2.PCV)
