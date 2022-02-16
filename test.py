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

feature_df = pd.read_excel('./sampleInput.xlsx')

intersection_test = intersection(feature_df)

print("PCV values: ", intersection_test.PCV)
print("PSI death risk values: ", intersection_test.PSI_death)
print("PSI severe injury risk values: ", intersection_test.PSI_injury)

