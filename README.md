# Data Acquisition Program for Multi-Sensor Multi-Hazard Monitoring Unmanned Aircraft System (mUAS)

A python-based data acquisition program for a remote sensing platform integrating multiple sensor payload on a unmanned aerial vehicle (UAV) for structural behavior monitoring and environmental feature collection under multi-hazards.

![mUAS_DAQ_Figure](https://github.com/user-attachments/assets/e95ad494-ce06-46f2-abf9-4e5ebdd08c59)

<p align='center'>
<b><i>Figure.</i></b> <i>Multi-sensor multi-hazard monitoring unmanned aircraft system (mUAS): 
(a) AutoCAD rendering of the mUAS, 
(b) Complete assembly of the mUAS at the test site,
(c) Various sensing payloads applied on the mUAS.</i>
</p>
 
For more details about the design of the mUAS and the data/results collected by this unique platform, please refer to the following publications:

1. Ji, R.; Lo, E.; Norton, T.J.; Driscoll, J.W.; Lozano Bravo, H.; Kuester, F.; Hutchinson, T.C. (2026). Unmanned Aircraft Systems (UAS) for Perishable Data Collection: Planning, Operations, and Inspection Practices. Structural Systems Research Project (SSRP) 2026-01. University of California San Diego. In review, link will be released after publication.
2. More publications in preparation...

### Python version:
3.10.11

### Required packages:
numpy, opencv, pyserial, depthai

### Test platform:
1. Ultrasonic anemometer x 2: LI-COR LI550-P
2. Particulate matter sensor x 1: Honeywell HPMA115C0-004
3. RGB camera x 1: Luxonis OAK-1W IMX378
4. Onboard computer: Intel NUC7i5BNH
5. UAS/UAV platform: Freefly Alta X (with Tattu 12S 16Ah battery set)
