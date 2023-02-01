#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version ='3.0'
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
        self.test_validity(feature_dict)
        self.PCV = self.getPotentialConflictVolume(feature_dict)
        self.PPP = self.getPresentPedestrianProbability(feature_dict)
        self.CS = self.getConflictSpeed(feature_dict)
        self.DR = self.getDeathRisk()
        self.SIR = self.getSevereInjuryRisk()
        self.PSI_death = self.getPedestrianRiskIndex('death')
        self.PSI_injury = self.getPedestrianRiskIndex('injury')
        # print('Crossing is initialized!')
    
    def getPotentialConflictVolume(self, feature_dict):
        """Computes potential conflict volumes.

        Args:
            feature_dict: A dictionary of features realted to geometric and signal plan

        Returns:
            A list of potential conflict volumes in the order of:

            [PCV_RT1_a, PCV_RT1_c, PCV_RT2_b, PCV_RT2_d, PCV_LT3_a]

        """
        ### Computing PCV_RT1_a and PCV_RT1_c
        S_base = feature_dict['baseSaturationFlow']
        c = feature_dict['cycleTime']
        W = feature_dict['pedWalkSpeed']

        ### Computing PCV_RT1_a and PCV_RT1_c
        if feature_dict['slipLane1']: 
            # if there is exclusive RT and slip lane on appraoch 1
            PCV_RT1_a = 0
            PCV_RT1_c = feature_dict['volume_RT1']
        else: 
            # if there isn't slip lane on appraoch 1
            PCV_RT1_c = 0

            ## 1) compute volume RTOR for appraoch 1
            if feature_dict['RTOR1'] == False or feature_dict['shoulderType1'] == 2:
                # if RTOT is not permitted or there is no RT or RT/TH lanes
                RT1_rtor = 0
            else:
                # compute q'ped for approach 1
                LPI = feature_dict['leadingPedInterval2']
                W_plus_FDW = feature_dict['walkInterval2']+feature_dict['flashingDontWalkInterval2']
                V_P1 = feature_dict['volume_P1']
                V_P1 = V_P1*c/3600 if LPI == 0 else max(0, V_P1*c/3600 - V_P1 * (c - W_plus_FDW)/3600)
                V_P1 = V_P1*3600/c
                q_prime_ped = V_P1 * c / (W_plus_FDW - LPI) 
                # compute F_Rped and F_redius
                F_Rped = max(0.49 - q_prime_ped/10645, 0) if q_prime_ped >= 200 else 1
                F_radius = max(0.5 + feature_dict['rightTurnRadius4']/30, 0) if feature_dict['rightTurnRadius4']<15 else 1
                # compute K_R
                S_R = S_base * min(F_Rped, F_radius)
                K_R = S_base/S_R if feature_dict['shoulderType4'] == 0 else 0
                # compute q'm and q_rtor
                V_TH4 = feature_dict['volume_TH4']
                V_RT4 = feature_dict['volume_RT4']
                N = feature_dict['laneNumber4']
                q_prime =  V_TH4 + V_RT4 * K_R
                
                q_m1 = max(0, q_prime/N - V_RT4*K_R) if feature_dict['shoulderType4'] == 0 else q_prime/N
                q_mR = V_RT4 if feature_dict['shoulderType4'] == 0 else 0

                q_prime_m1 = q_mR * c / feature_dict['effectiveRed1']
                q_prime_mR = q_m1 * c / feature_dict['effectiveRed1']


                q_prime_m = q_prime_mR/2 + q_prime_m1

                q_rtor = 850 - 0.35 * q_prime_m

                # compute C_rtor_exc
                tw1 = 5.25/W
                V_P1 = feature_dict['volume_P1']
                f_Pb = min(1, tw1*V_P1/3600) 
                P_b = 1 - f_Pb
                C_rtor_exc = P_b * q_rtor * feature_dict['effectiveRed1'] / c

                # compute C_rtor
                if feature_dict['shoulderType1'] == 1:
                    # if RT1 has exclusive lane
                    C_rtor = C_rtor_exc
                else:
                    LPI = feature_dict['leadingPedInterval1']
                    W_plus_FDW = feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1']
                    V_P2 = feature_dict['volume_P2']
                    V_P2 = V_P2*c/3600 if LPI == 0 else max(0, V_P2*c/3600 - V_P2*(c - W_plus_FDW)/3600)
                    V_P2 = V_P2*3600/c

                    q_prime_ped = V_P2 * c / (W_plus_FDW - LPI)

                    F_Rped = max(0.49-q_prime_ped/10645,0) if q_prime_ped >= 200 else 1.0
                    F_radius = max(0.5 + feature_dict['rightTurnRadius1']/30, 0) if feature_dict['rightTurnRadius1']<15 else 1
                    S_R = S_base * min(F_Rped, F_radius)
                    K_R = S_base/S_R if feature_dict['shoulderType1'] == 0 else 0

                    V_TH1 = feature_dict['volume_TH1']
                    V_RT1 = feature_dict['volume_RT1']
                    N = feature_dict['laneNumber1']
                    q_prime =  V_TH1 + V_RT1 * K_R
                    V_R = V_RT1
                    V_T = max(0, q_prime/N - V_RT1*K_R) if feature_dict['shoulderType1'] == 0 else q_prime/N

                    P_R = V_R / (V_R + V_T)
                    f_hat = P_R * P_R ** (q_rtor*feature_dict['effectiveRed1']/10600)

                    C_rtor = C_rtor_exc * f_hat
                
                q_rtor_arrival = feature_dict['volume_RT1'] * feature_dict['effectiveRed1'] / c 
                RT1_rtor = min(C_rtor, q_rtor_arrival)

            ## 2) compute RT in protected phase
            if feature_dict['effectiveGreenProtectedRightTurn1'] == 0:
                # if there is no protected RT phase
                RT1_protected = 0
            else:
                q_arrive_g_protect = feature_dict['volume_RT1'] * feature_dict['effectiveGreenProtectedRightTurn1'] / c 
                q_arrive_ROR = feature_dict['volume_RT1'] * feature_dict['effectiveRed1'] / c 
                q_arrive_protect = q_arrive_g_protect + q_arrive_ROR - RT1_rtor
                
                F_radius = max(0.5 + feature_dict['rightTurnRadius1']/30, 0) if feature_dict['rightTurnRadius1']<15 else 1
                C_protected = S_base * F_radius * feature_dict['effectiveGreenProtectedRightTurn1'] / c 

                RT1_protected = min(C_protected, q_arrive_protect)


            ## 3) compute conflicting volumes
            PCV_RT1_a = max(feature_dict['volume_RT1'] - RT1_rtor - RT1_protected, 0)
            PCV_RT1_a = PCV_RT1_a * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreenPermissive1'])
        
        ### Computing PCV_RT2_b and PCV_RT2_d
        if feature_dict['slipLane2']: 
            # if there is exclusive RT and slip lane on appraoch 2 
            PCV_RT2_b = 0
            PCV_RT2_d = feature_dict['volume_RT2']
        elif feature_dict['RTOR2']==False:
            # if right turn on red is not permitted
            PCV_RT2_b = 0
            PCV_RT2_d = 0
        else: 
            PCV_RT2_d = 0
            ## 1) compute volume RTOR for appraoch 2
            # compute q'ped for approach 1
            LPI = feature_dict['leadingPedInterval1']
            W_plus_FDW = feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1']
            V_P2 = feature_dict['volume_P2']
            V_P2 = V_P2*c/3600 if LPI == 0 else max(0, V_P2*c/3600 - V_P2*(c - W_plus_FDW)/3600)
            V_P2 = V_P2*3600/c
            q_prime_ped = V_P2 * c / (W_plus_FDW - LPI)
            # compute F_Rped and F_redius 
            F_Rped = max(0.49-q_prime_ped/10645.0,0) if q_prime_ped > 200 else 1.0
            F_radius = max(0.5 + feature_dict['rightTurnRadius1']/30, 0) if feature_dict['rightTurnRadius1']<15 else 1
            # compute q'm and q_rtor
            S_R = S_base * min(F_Rped, F_radius)
            K_R = S_base/S_R if feature_dict['shoulderType1'] == 0 else 0
            V_TH1 = feature_dict['volume_TH1']
            V_RT1 = feature_dict['volume_RT1']
            N = feature_dict['laneNumber1']
            q_prime =  V_TH1 + V_RT1 * K_R
            V_R = V_RT1 if feature_dict['shoulderType1'] == 0 else 0
            V_T = max(0, q_prime/N - V_RT1*K_R) if feature_dict['shoulderType1'] == 0 else q_prime/N
            q_prime_m1 = V_T * c / feature_dict['effectiveRed2']
            q_prime_mR = V_R * c / feature_dict['effectiveRed2']
            q_prime_m = q_prime_mR/2 + q_prime_m1

            q_rtor = 850 - 0.35 * q_prime_m

            # compute C_rtor_exc for RT2
            tw2 = 5.25/W
            V_P2 = feature_dict['volume_P2']
            f_Pb = min(1, tw2*V_P2/3600) 
            P_b = 1 - f_Pb
            C_rtor_exc = P_b * q_rtor * feature_dict['effectiveRed2'] / c

            # compute C_rtor for RT2
            if feature_dict['shoulderType2'] == 1:
                # if RT2 has exclusive lane
                C_rtor = C_rtor_exc
            else:
                LPI = feature_dict['leadingPedInterval2']
                W_plus_FDW = feature_dict['walkInterval2']+feature_dict['flashingDontWalkInterval2']
                V_P3 = feature_dict['volume_P3']
                V_P3 = V_P3*c/3600 if LPI == 0 else max(0, V_P3*c/3600 - V_P3*(c - W_plus_FDW)/3600)
                V_P3 = V_P3*3600/c
                q_prime_ped = V_P3 * c / (W_plus_FDW - LPI)

                F_Rped = max(0.49-q_prime_ped/10645,0) if q_prime_ped >= 200 else 1.0
                F_radius = max(0.5 + feature_dict['rightTurnRadius2']/30, 0) if feature_dict['rightTurnRadius2']<15 else 1
                S_R = S_base * min(F_Rped, F_radius)
                K_R = S_base/S_R if feature_dict['shoulderType2'] == 0 else 0

                V_TH2 = feature_dict['volume_TH2']
                V_RT2 = feature_dict['volume_RT2']
                N = feature_dict['laneNumber2']
                q_prime =  V_TH2 + V_RT2 * K_R
                
                V_R = V_RT2
                V_T = max(0, q_prime/N - V_RT2*K_R) if feature_dict['shoulderType2'] == 0 else q_prime/N

                P_R = V_R / (V_R + V_T)
                f_hat = P_R * P_R ** (q_rtor*feature_dict['effectiveRed2']/10600)
                C_rtor = C_rtor_exc * f_hat
            
            q_rtor_arrival = feature_dict['volume_RT2'] * feature_dict['effectiveRed2'] / c 
            RT2_rtor = min(C_rtor, q_rtor_arrival)
            ## 2) compute conflicting volumes
            PCV_RT2_b = RT2_rtor * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveRed2'])

        ### Computing PCV_LT3_a
        if feature_dict['leftTurnType3'] == "permissive":
            PCV_LT3_a = feature_dict['volume_LT3'] * (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreenPermissive3']
        elif feature_dict['leftTurnType3'] == "protected":
            PCV_LT3_a = 0
        else: # protected and permissive
            ### during protected phase
            ## a.1: compute max number of LT vehicles that can discharge during protected phase
            max_discharged_protected = feature_dict['baseSaturationFlow'] * feature_dict['effectiveGreenProtectedLeftTurn3']/3600
            ## a.2: compute # veh waiting to be served assuming uniform arrivals
            re = feature_dict['cycleTime'] - feature_dict['effectiveGreenProtectedLeftTurn3'] - feature_dict['effectiveGreenPermissive3']
            wating_veh = feature_dict['volume_LT3'] * re / 3600
            ## a.3: # vehicle served is min of arrivals and capacity
            served_veh_protected = min(wating_veh, max_discharged_protected) * 3600 / feature_dict['cycleTime']

            ### during permissive phase
            ## a.1: Determine vehicles that will be served during permissive phase
            served_veh_permissive = feature_dict['volume_LT3'] - served_veh_protected
            PCV_LT3_a = served_veh_permissive * min(1, (feature_dict['walkInterval1']+feature_dict['flashingDontWalkInterval1'])/feature_dict['effectiveGreenPermissive3'])
        
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
        tw_a = feature_dict['width_a2']/feature_dict['pedWalkSpeed']
        tw_b = feature_dict['width_b2']/feature_dict['pedWalkSpeed']
        tw_c = feature_dict['width_c2']/feature_dict['pedWalkSpeed']
        tw_d = feature_dict['width_d2']/feature_dict['pedWalkSpeed']
        
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

    def test_validity(self, feature_dict):
        if feature_dict['laneNumber1'] == 1:
            if feature_dict['shoulderType1'] == 1:
                print('Shoulder Type: ', feature_dict['shoulderType1'], 
                      'Slip Lane: ', feature_dict['slipLane1'],
                      'Lane Number: ', feature_dict['laneNumber1'])
                raise Exception('Error: Shoulder type can never equal 1 (because then through vehicle cannot proceed) when lane Number = 1')
                
            if feature_dict['shoulderType1'] == 2 and feature_dict['slipLane1'] == False:
                print('Shoulder Type: ', feature_dict['shoulderType1'], 
                      'Slip Lane: ', feature_dict['slipLane1'],
                      'Lane Number: ', feature_dict['laneNumber1'])
                raise Exception('Error: Shoulder type can never equal 1 when lane Number = 1 and there is no a slip lane ')
        if feature_dict['laneNumber1'] >= 2:
            if feature_dict['shoulderType1'] == 2 and feature_dict['slipLane1'] == False:
                print('Shoulder Type: ', feature_dict['shoulderType1'], 
                      'Slip Lane: ', feature_dict['slipLane1'],
                      'Lane Number: ', feature_dict['laneNumber1'])
                raise Exception('Error: Shoulder type can never equal 2 when lane Number >= 2 and there is no a slip lane ')
        if feature_dict['slipLane1'] == True and feature_dict['shoulderType1'] != 2:
            print('Shoulder Type: ', feature_dict['shoulderType1'], 
                  'Slip Lane: ', feature_dict['slipLane1'],
                  'Lane Number: ', feature_dict['laneNumber1'])
            raise Exception('Error: Shoulder type can only equal 2 when there is a slip lane ')
        