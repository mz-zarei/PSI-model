#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version ='1.0'
# ---------------------------------------------------------------------------
"""Implementation of crossing class for pedestrian risk index model"""
# ---------------------------------------------------------------------------
# Imports
import math

class Crossing:
    """represents a crossing of a 4-legged intersection.

    This crossing reperents #2 in the provided guidline and all numbers in the feature input
    should follow the same format.
                

    Attributes:
        PCV List[float]: Potential conflicting volumes - [PCV_RT1_a, PCV_RT1_c, PCV_RT2_b, PCV_RT2_d, PCV_LT3_a]
        PPP List[float]: Probability of pedestrain being present in the crossing - [PPP_RT1_a, PPP_RT1_b, PPP_RT2_c, PPP_RT2_d, PPP_LT3_a]
        CS List[float]: Conflict speeds - [CS_RT1_a, CS_RT1_c, CS_RT2_b, CS_RT2_d, CS_LT3_a]
        DR List[float]: Death risk for potential crash - [DR_RT1_a, DR_RT1_c, DR_RT2_b, DR_RT2_d, DR_LT3_a]
        SIR List[float]: Severe injury risk for potential crash - [SIR_RT1_a, SIR_RT1_c, SRI_RT2_b, SIR_RT2_d, SIR_LT3_a]
        PSI_death float: Pedstrian safety index using death risk model
        PSI_injury float: Pedstrian safety index using severe injury risk model

    """

    def __init__(self, feature_dict):
        
        self.PCV = self.getPotentialConflictVolume(feature_dict)
        self.PPP = self.getPresentPedestrianProbability(feature_dict)
        self.CS = self.getConflictSpeed(feature_dict)
        self.DR = self.getDeathRisk()
        self.SIR = self.getSevereInjuryRisk()
        self.PSI_death = self.getPedestrianRiskIndex('death')
        self.PSI_injury = self.getPedestrianRiskIndex('injury')
        print('Crossing is initialized!')
    
    def getPotentialConflictVolume(self, feature_dict):
        """Computes potential conflict volumes.

        Args:
            feature_dict: A dictionary of features realted to geometric and signal plan

        Returns:
            A list of potential conflict volumes in the order of:

            [PCV_RT1_a, PCV_RT1_c, PCV_RT2_b, PCV_RT2_d, PCV_LT3_a]

        """
        ### Computing PCV_RT1_a and PCV_RT1_c

        if feature_dict['slipLane1']: 
            # if there is slip lane on appraoch 1
            PCV_RT1_a = 0
            PCV_RT1_c = feature_dict['volume_RT1']
        else: 
            # if there isn't slip lane on appraoch 1
            PCV_RT1_c = 0
            if (feature_dict['RTOR1']) and (feature_dict['shoulderType1'] == 1):
                # if RTOR is permitted and RT is in exclusive lane on appraoch 1
                if feature_dict['slipLane4']:
                    # if there is right turn slip lane on approach 4
                                        
                    ## b) compute the maximum flow (capacity) that can be served by the RTOR (process from CCG)
                    q_m1 = feature_dict['volume_TH4']/feature_dict['laneNumber4'] * feature_dict['cycleTime']/feature_dict['effectiveRed1']
                    q_mR = 0
                    q_prime_m1 = q_m1 * feature_dict['cycleTime']/feature_dict['effectiveRed1']
                    q_prime_mR = q_mR * feature_dict['cycleTime']/feature_dict['effectiveRed1']
                    q_prime_m = q_prime_mR/2 + q_prime_m1
                    q_RTOR = 850 - 0.35 * q_prime_m
                    

                    ## c) number of vehicles on approach 1 that will discharge is minimum of q_arrival and q_RTOR
                    min_qArrival_qRTOR = min(feature_dict['volume_RT1']*feature_dict['effectiveRed1']/3600,
                                             q_RTOR*feature_dict['effectiveRed1']/3600) * 3600/feature_dict['cycleTime']
                    volume_RT1_prime = feature_dict['volume_RT1'] - min_qArrival_qRTOR
                    PCV_RT1_a = volume_RT1_prime * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreen1'])
                else:
                    # if there isn't right turn slip lane on approach 4  
                    ## a) compute conflicting volume in shoulder lane on approach 4 (requires allocation of the through volume to the available lanes)
                    # compute pedstrian volume during the cycle length
                    q_ped = feature_dict['volume_P1']*feature_dict['cycleTime']/(feature_dict['walkInterval2']+feature_dict['flashingDontWalkInterval2']-feature_dict['leadingPedInterval2'])
                    # the adjustment factor for the right turns on approach 4 by accounting for volume_P1.
                    F_Rped = max(feature_dict['a_Frped'] -  q_ped/feature_dict['b_Frped'], 0)
                    # the saturation flow rate (SR) for the approach
                    SR = feature_dict['baseSaturationFlow'] * F_Rped
                    KR = 1/F_Rped
                    # equivalent thru volume of thru movement & right turn movement approach 4 
                    q_prime = feature_dict['volume_TH4'] * 1.0 + feature_dict['volume_RT4'] * KR
                    # right turn volume in shoulder lane:
                    q_mR = feature_dict['volume_RT4']
                    # check if right turn lane operates as exclusive through lane
                    if q_prime/feature_dict['laneNumber4'] < feature_dict['volume_RT4'] * KR:
                        # through volume in shoulder lane
                        q_m1 = 0
                    else:
                        # if the exclusicve right turn lane operates as shared lane
                        q_m1 = feature_dict['volume_TH4'] - (feature_dict['laneNumber4']-1) * q_prime/feature_dict['laneNumber4']
                    
                    ## b) compute the maximum flow (capacity) that can be served by the RTOR (process from CCG)
                    q_prime_m1 = q_m1 * feature_dict['cycleTime']/feature_dict['effectiveRed1']
                    q_prime_mR = q_mR * feature_dict['cycleTime']/feature_dict['effectiveRed1']
                    q_prime_m = q_prime_mR/2 + q_prime_m1
                    q_RTOR = 850 - 0.35 * q_prime_m

                    ## c) number of vehicles on approach 1 that will discharge is minimum of q_arrival and q_RTOR
                    min_qArrival_qRTOR = min( feature_dict['volume_RT1']*feature_dict['effectiveRed1']/3600,
                                              q_RTOR*feature_dict['effectiveRed1']/3600) * 3600/feature_dict['cycleTime']
                    volume_RT1_prime = feature_dict['volume_RT1'] - min_qArrival_qRTOR
                    PCV_RT1_a = volume_RT1_prime * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreen1'])
            else:
                # if (1) RTOR isn't permitted, or (2) RT isn't in exclusive lane on appraoch 1
                PCV_RT1_a = feature_dict['volume_RT1'] * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreen1'])

        ### Computing PCV_RT2_b and PCV_RT2_d
        if feature_dict['slipLane2']: 
            # if there is slip lane on appraoch 2
            PCV_RT2_b = 0
            PCV_RT2_d = feature_dict['volume_RT2']
        else: 
            # if there isn't slip lane on appraoch 2
            PCV_RT2_d = 0
            if (feature_dict['RTOR2']) and (feature_dict['shoulderType2'] == 1):
                # if RTOR is permitted and RT is in exclusive lane on appraoch 2
                if feature_dict['slipLane1']:
                    # if there is right turn slip lane on approach 1
                                        
                    ## b) compute the maximum flow (capacity) that can be served by the RTOR (process from CCG)
                    q_m1 = feature_dict['volume_TH1']/feature_dict['laneNumber1']
                    q_mR = 0
                    q_prime_m1 = q_m1 * feature_dict['cycleTime']/feature_dict['effectiveRed2']
                    q_prime_mR = q_mR * feature_dict['cycleTime']/feature_dict['effectiveRed2']
                    q_prime_m = q_prime_mR/2 + q_prime_m1
                    q_RTOR = 850 - 0.35 * q_prime_m
                    ## c) number of RT vehicles on approach 2 that will discharge is minimum of q_arrival and q_RTOR
                    min_qArrival_qRTOR = min(feature_dict['volume_RT2'],q_RTOR) * feature_dict['effectiveRed2']/3600
                    volume_RT2_prime = min_qArrival_qRTOR * 3600/feature_dict['cycleTime']
                    PCV_RT2_b = volume_RT2_prime * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveRed2'])
                else:
                    # if there isn't right turn slip lane on approach 1 
                    ## a) compute conflicting volume in shoulder lane on approach 1 (requires allocation of the through volume to the available lanes)
                    # compute pedstrian volume during the cycle length
                    q_ped = feature_dict['volume_P2']*feature_dict['cycleTime']/(feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1']-feature_dict['leadingPedInterval1'])
                    # the adjustment factor for the right turns on approach 1 by accounting for volume_P2.
                    F_Rped = 1 if q_ped <200 else max(feature_dict['a_Frped'] -  q_ped/feature_dict['b_Frped'], 0)
                    # the saturation flow rate (SR) for the approach
                    SR = feature_dict['baseSaturationFlow'] * F_Rped
                    KR = 1/F_Rped
                    # equivalent thru volume of thru movement & right turn movement approach 4 
                    q_prime = feature_dict['volume_TH1'] * 1.0 + feature_dict['volume_RT1'] * KR
                    # right turn volume in shoulder lane:
                    q_mR = feature_dict['volume_RT1']
                    # check if right turn lane operates as exclusive through lane
                    if q_prime/feature_dict['laneNumber1'] < feature_dict['volume_RT1'] * KR:
                        # through volume in shoulder lane
                        q_m1 = 0
                    else:
                        # if the exclusicve right turn lane operates as shared lane
                        q_m1 = feature_dict['volume_TH1'] - (feature_dict['laneNumber1']-1) * q_prime/feature_dict['laneNumber1']
                    
                    ## b) compute the maximum flow (capacity) that can be served by the RTOR (process from CCG)
                    q_prime_m1 = q_m1 * feature_dict['cycleTime']/feature_dict['effectiveRed2']
                    q_prime_mR = q_mR * feature_dict['cycleTime']/feature_dict['effectiveRed2']
                    q_prime_m = q_prime_mR/2 + q_prime_m1
                    q_RTOR = 850 - 0.35 * q_prime_m
                    
                    ## c) number of vehicles on approach 1 that will discharge is minimum of q_arrival and q_RTOR
                    min_qArrival_qRTOR = min( feature_dict['volume_RT2']*feature_dict['effectiveRed2']/3600,
                                              q_RTOR*feature_dict['effectiveRed2']/3600) * 3600/feature_dict['cycleTime']
                    volume_RT2_prime = min_qArrival_qRTOR
                    print(min_qArrival_qRTOR)
                    PCV_RT2_b = volume_RT2_prime * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveRed2'])
            else:
                # if (1) RTOR isn't permitted, or (2) RT isn't in exclusive lane on appraoch 2
                PCV_RT2_b = 0
                

        ### Computing PCV_LT3_a
        if feature_dict['leftTurnType3'] == "permissive":
            PCV_LT3_a = feature_dict['volume_LT3'] * (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreen1']
        elif feature_dict['leftTurnType3'] == "protected":
            PCV_LT3_a = 0
        else: # protected and permissive
            ### during protected phase
            ## a.1: compute max number of LT vehicles that can discharge during protected phase
            max_discharged_protected = feature_dict['baseSaturationFlow'] * feature_dict['effectiveGreenProtected1']/3600
            ## a.2: compute # veh waiting to be served assuming uniform arrivals
            re = feature_dict['cycleTime'] - feature_dict['effectiveGreenProtected1'] - feature_dict['effectiveGreen1']
            wating_veh = feature_dict['volume_LT3'] * re / 3600
            ## a.3: # vehicle served is min of arrivals and capacity
            served_veh_protected = min(wating_veh, max_discharged_protected) * 3600 / feature_dict['cycleTime']

            ### during permissive phase
            ## a.1: Determine vehicles that will be served during permissive phase
            served_veh_permissive = feature_dict['volume_LT3'] - served_veh_protected
            PCV_LT3_a = served_veh_permissive * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreen1'])
        
        res = [PCV_RT1_a, PCV_RT1_c, PCV_RT2_b, PCV_RT2_d, PCV_LT3_a]
        res = [round(i, 3) for i in res]
        return res

    def getPresentPedestrianProbability(self, feature_dict):
        """Computes probability of pedestrain being present in the crossing

        Args:
            feature_dict: A dictionary of features realted to geometric and signal plan

        Returns:
            A list of present pedestrian probabilities in the order of:

            [PPP_RT1_a, PPP_RT1_c, PPP_RT2_b, PPP_RT2_d, PPP_LT3_a]

        """
        ## Compute required time for peds to pass the area a,b,c,d
        tw_a = feature_dict['width_a']/feature_dict['pedWalkSpeed']
        tw_b = feature_dict['width_b']/feature_dict['pedWalkSpeed']
        tw_c = feature_dict['width_c']/feature_dict['pedWalkSpeed']
        tw_d = feature_dict['width_d']/feature_dict['pedWalkSpeed']
        
        ## Compute average ped headway for area a, b
        effectivePedVolume_ab = feature_dict['volume_P2'] * feature_dict['cycleTime'] / (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1']-feature_dict['leadingPedInterval1'])
        pedHeadway_ab = 3600 / effectivePedVolume_ab
        
        ## Compute average ped headway for area c, d (assume peds cross at anytime during cycle; not just green)
        effectivePedVolume_cd = feature_dict['volume_P2'] 
        pedHeadway_cd = 3600 / effectivePedVolume_cd

        ## Compute probability of present ped in the areas
        PPP_RT1_a = 1 - math.exp(-tw_a/pedHeadway_ab)
        PPP_RT1_c = 1 - math.exp(-tw_c/pedHeadway_cd) if feature_dict['slipLane1']  else 0
        PPP_RT2_b = 1 - math.exp(-tw_b/pedHeadway_ab)
        PPP_RT2_d = 1 - math.exp(-tw_d/pedHeadway_cd) if feature_dict['slipLane2']  else 0
        PPP_LT3_a = 1 - math.exp(-tw_a/pedHeadway_ab)

        res = [PPP_RT1_a, PPP_RT1_c, PPP_RT2_b, PPP_RT2_d, PPP_LT3_a]
        res = [round(i, 3) for i in res]
        return res

    def getConflictSpeed(self, feature_dict):
        """Computes conflicting speeds in the crossing areas

        Args:
            feature_dict: A dictionary of features realted to geometric and signal plan

        Returns:
            A list of conflicting speeds in the order of:

            [CS_RT1_a, CS_RT1_c, CS_RT2_b, CS_RT2_d, CS_LT3_a]

        """
        ## RT model coefs
        o = 2.465682
        a, Iy = 0.0471218, 0
        b, ITk = -0.1428277, 0
        c =	0.0035318
        d =	-0.1375053
        e, IThru =	0.8183215, 0
        f =	0.032
        g =	0.0076864
        z = 1.0364              # for 85th percentile

        ## Compute right turn speeds on apprrach 2
        CS_RT2_b = 8                                         # assumed (km/h)
        rRT2 = feature_dict['rightTurnRadius2'] * 3.28       # radius of right turn in ft
        if self.PCV[3] != 0:
            tH = 3600 / self.PCV[3]                              # time headway between preceeding vehicle and vehicle of interest (seconds)                       
            CS_RT2_d = math.exp(o + a*Iy + b*ITk + c*rRT2 + (d + e*IThru + f*rRT2 + g*rRT2*IThru )/tH**2 + z*0.19)
        else:
            CS_RT2_d = 0

        ## Compute right turn speeds on apprrach 1
        rRT1 = feature_dict['rightTurnRadius1'] * 3.28       # radius of right turn in ft
        if feature_dict['volume_RT1'] != 0:
            tH = 3600 / feature_dict['volume_RT1']               # time headway between preceeding vehicle and vehicle of interest (seconds)
            CS_RT1_a = math.exp(o + a*Iy + b*ITk + c*rRT1 + (d + e*IThru + f*rRT1 + g*rRT1*IThru )/tH**2 + z*0.19)
        else:
            CS_RT1_a = 0
        if self.PCV[1] != 0:
            tH = 3600 / self.PCV[1]
            CS_RT1_c = math.exp(o + a*Iy + b*ITk + c*rRT1 + (d + e*IThru + f*rRT1 + g*rRT1*IThru )/tH**2 + z*0.19)
        else:
            CS_RT1_c = 0
        ## Compute left turn speeds from apprach 3 using LT radius and corrected AASHTO Model
        correction_factor, f_r = 1.38, 0.16                  
        CS_LT3_a = correction_factor * math.sqrt(127 * feature_dict['leftTurnRadius3'] * f_r)
        
        res = [CS_RT1_a, CS_RT1_c, CS_RT2_b, CS_RT2_d, CS_LT3_a]
        res = [round(i,3) for i in res]
        return res

    def getDeathRisk(self):
        """Computes the probability of a crash being fatal in the crossing areas

        Returns:
            A list of death risks for potential crashes in the order of:

            [DR_RT1_a, DR_RT1_c, DR_RT2_b, DR_RT2_d, DR_LT3_a]

        """
        k = 6E-07
        n = 3.35
        DR_model = lambda s: 1-math.exp(-k*(s**n))

        # DR_list: [DR_RT1_a, DR_RT1_c, DR_RT2_b, DR_RT2_d, DR_LT3_a]
        DR_list = [round(DR_model(s),3) for s in self.CS]

        return DR_list
    
    def getSevereInjuryRisk(self):
        """Computes the probability of a crash being severe injury in the crossing areas

        Args:
            speedList: A list of speeds for each area of crossing, [CS_RT1_a, CS_RT1_c, CS_RT2_b, CS_RT2_d, CS_LT3_a]

        Returns:
            A list of death risks for potential crashes in the order of:

            [SIR_RT1_a, SIR_RT1_c, SIR_RT2_b, SIR_RT2_d, SIR_LT3_a]

        """
        k = 1.7E-06
        n = 3.25
        SIR_model = lambda s: 1-math.exp(-k*(s**n))

        # SIR_list: [SIR_RT1_a, SIR_RT1_c, SIR_RT2_b, SIR_RT2_d, SIR_LT3_a]
        SIR_list = [round(SIR_model(s),3) for s in self.CS]

        return SIR_list

    def getPedestrianRiskIndex(self, severity):
        """Computes the pedestrian risk index for the crossing

        Args:
            severity: determine which severity model to use 'death' or 'injury'

        Returns:
            A list of Pedstrian safety index for potential crash potential crashes in the order of:

            [PSI_RT1_a, PSI_RT1_b, PSI_RT2_c, PSI_RT2_d, PSI_LT3_a]

        """
        if severity == 'death':
            PSI = sum([a*b*c for a,b,c in zip(self.PCV, self.PPP, self.DR)])
        if severity == 'injury':
            PSI = sum([a*b*c for a,b,c in zip(self.PCV, self.PPP, self.SIR)])
        return round(PSI, 3)

