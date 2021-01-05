"""Android USB host serial vendor ids and product ids.

FTDI_VENDOR_ID
SILABS_VENDOR_ID
QINHENG_VENDOR_ID
PROLIFIC_VENDOR_ID

FTDI_VID_PID_GROUP
SILABS_VID_PID_GROUP
QINHENG_VID_PID_GROUP
PROLIFIC_VID_PID_GROUP
"""

FTDI_VENDOR_ID = 0x0403
SILABS_VENDOR_ID = 0x10C4
QINHENG_VENDOR_ID = 0x1A86
PROLIFIC_VENDOR_ID = 0x067B

FTDI_VID_PID_GROUP = [
    (0x03EB, 0x2109),
    (0x0456, 0xF000),
    (0x0456, 0xF001),
    (0x04D8, 0x000A),
    (0x0584, 0xB020),
    (0x0647, 0x0100),
    (0x06CE, 0x8311),
    (0x06D3, 0x0284),
    (0x0856, 0xAC01),
    (0x0856, 0xAC02),
    (0x0856, 0xAC03),
    (0x0856, 0xAC11),
    (0x0856, 0xAC12),
    (0x0856, 0xAC16),
    (0x0856, 0xAC17),
    (0x0856, 0xAC18),
    (0x0856, 0xAC19),
    (0x0856, 0xAC25),
    (0x0856, 0xAC26),
    (0x0856, 0xAC27),
    (0x0856, 0xAC33),
    (0x0856, 0xAC34),
    (0x0856, 0xAC49),
    (0x0856, 0xAC50),
    (0x0856, 0xBA02),
    (0x093C, 0x0601),
    (0x093C, 0x0701),
    (0x0ACD, 0x0300),
    (0x0B39, 0x0103),
    (0x0B39, 0x0421),
    (0x0C26, 0x0004),
    (0x0C26, 0x0018),
    (0x0C26, 0x0009),
    (0x0C26, 0x000A),
    (0x0C26, 0x000B),
    (0x0C26, 0x000C),
    (0x0C26, 0x000D),
    (0x0C26, 0x0010),
    (0x0C26, 0x0011),
    (0x0C26, 0x0012),
    (0x0C26, 0x0013),
    (0x0C33, 0x0010),
    (0x0C52, 0x2101),
    (0x0C52, 0x2101),
    (0x0C52, 0x2102),
    (0x0C52, 0x2103),
    (0x0C52, 0x2104),
    (0x0C52, 0x9020),
    (0x0C52, 0x2211),
    (0x0C52, 0x2221),
    (0x0C52, 0x2212),
    (0x0C52, 0x2222),
    (0x0C52, 0x2213),
    (0x0C52, 0x2223),
    (0x0C52, 0x2411),
    (0x0C52, 0x2421),
    (0x0C52, 0x2431),
    (0x0C52, 0x2441),
    (0x0C52, 0x2412),
    (0x0C52, 0x2422),
    (0x0C52, 0x2432),
    (0x0C52, 0x2442),
    (0x0C52, 0x2413),
    (0x0C52, 0x2423),
    (0x0C52, 0x2433),
    (0x0C52, 0x2443),
    (0x0C52, 0x2811),
    (0x0C52, 0x2821),
    (0x0C52, 0x2831),
    (0x0C52, 0x2841),
    (0x0C52, 0x2851),
    (0x0C52, 0x2861),
    (0x0C52, 0x2871),
    (0x0C52, 0x2881),
    (0x0C52, 0x2812),
    (0x0C52, 0x2822),
    (0x0C52, 0x2832),
    (0x0C52, 0x2842),
    (0x0C52, 0x2852),
    (0x0C52, 0x2862),
    (0x0C52, 0x2872),
    (0x0C52, 0x2882),
    (0x0C52, 0x2813),
    (0x0C52, 0x2823),
    (0x0C52, 0x2833),
    (0x0C52, 0x2843),
    (0x0C52, 0x2853),
    (0x0C52, 0x2863),
    (0x0C52, 0x2873),
    (0x0C52, 0x2883),
    (0x0C52, 0xA02A),
    (0x0C52, 0xA02B),
    (0x0C52, 0xA02C),
    (0x0C52, 0xA02D),
    (0x0C6C, 0x04B2),
    (0x0C7D, 0x0005),
    (0x0D3A, 0x0300),
    (0x0D46, 0x2020),
    (0x0D46, 0x2021),
    (0x0DCD, 0x0001),
    (0x0F94, 0x0001),
    (0x0F94, 0x0005),
    (0x0FD8, 0x0001),
    (0x103E, 0x03E8),
    (0x104D, 0x3000),
    (0x104D, 0x3002),
    (0x104D, 0x3006),
    (0x1209, 0x1002),
    (0x1209, 0x1006),
    (0x128D, 0x0001),
    (0x1342, 0x0202),
    (0x1457, 0x5118),
    (0x15BA, 0x0003),
    (0x15BA, 0x002B),
    (0x1781, 0x0C30),
    (0x2100, 0x9001),
    (0x2100, 0x9E50),
    (0x2100, 0x9E51),
    (0x2100, 0x9E52),
    (0x2100, 0x9E53),
    (0x2100, 0x9E54),
    (0x2100, 0x9E55),
    (0x2100, 0x9E56),
    (0x2100, 0x9E57),
    (0x2100, 0x9E58),
    (0x2100, 0x9E59),
    (0x2100, 0x9E5A),
    (0x2100, 0x9E5B),
    (0x2100, 0x9E5C),
    (0x2100, 0x9E5D),
    (0x2100, 0x9E5E),
    (0x2100, 0x9E5F),
    (0x2100, 0x9E60),
    (0x2100, 0x9E61),
    (0x2100, 0x9E62),
    (0x2100, 0x9E63),
    (0x2100, 0x9E64),
    (0x2100, 0x9E65),
    (0x2100, 0x9E65),
    (0x2100, 0x9E66),
    (0x2100, 0x9E67),
    (0x2100, 0x9E68),
    (0x2100, 0x9E69),
    (0x2100, 0x9E6A),
    (0x1A72, 0x1000),
    (0x1A72, 0x1001),
    (0x1A72, 0x1002),
    (0x1A72, 0x1005),
    (0x1A72, 0x1007),
    (0x1A72, 0x1008),
    (0x1A72, 0x1009),
    (0x1A72, 0x100D),
    (0x1A72, 0x100E),
    (0x1A72, 0x100F),
    (0x1A72, 0x1011),
    (0x1A72, 0x1012),
    (0x1A72, 0x1013),
    (0x1A72, 0x1014),
    (0x1A72, 0x1015),
    (0x1A72, 0x1016),
    (0x165C, 0x0002),
    (0x1A79, 0x6001),
    (0x1B3D, 0x0100),
    (0x1B3D, 0x0101),
    (0x1B3D, 0x0102),
    (0x1B3D, 0x0103),
    (0x1B3D, 0x0104),
    (0x1B3D, 0x0105),
    (0x1B3D, 0x0106),
    (0x1B3D, 0x0107),
    (0x1B3D, 0x0108),
    (0x1B3D, 0x0109),
    (0x1B3D, 0x010A),
    (0x1B3D, 0x010B),
    (0x1B3D, 0x010C),
    (0x1B3D, 0x010D),
    (0x1B3D, 0x010E),
    (0x1B3D, 0x010F),
    (0x1B3D, 0x0110),
    (0x1B3D, 0x0111),
    (0x1B3D, 0x0112),
    (0x1B3D, 0x0113),
    (0x1B3D, 0x0114),
    (0x1B3D, 0x0115),
    (0x1B3D, 0x0116),
    (0x1B3D, 0x0117),
    (0x1B3D, 0x0118),
    (0x1B3D, 0x0119),
    (0x1B3D, 0x011A),
    (0x1B3D, 0x011B),
    (0x1B3D, 0x011C),
    (0x1B3D, 0x011D),
    (0x1B3D, 0x011E),
    (0x1B3D, 0x011F),
    (0x1B3D, 0x0120),
    (0x1B3D, 0x0121),
    (0x1B3D, 0x0122),
    (0x1B3D, 0x0123),
    (0x1B3D, 0x0124),
    (0x1B3D, 0x0125),
    (0x1B3D, 0x0126),
    (0x1B3D, 0x0127),
    (0x1B3D, 0x0128),
    (0x1B3D, 0x0129),
    (0x1B3D, 0x012A),
    (0x1B3D, 0x012B),
    (0x1B3D, 0x012C),
    (0x1B3D, 0x012E),
    (0x1B3D, 0x012F),
    (0x1B3D, 0x0130),
    (0x1B91, 0x0064),
    (0x1BC9, 0x6001),
    (0x1C0C, 0x0102),
    (0x1CF1, 0x0001),
    (0x1CF1, 0x0041),
    (0x0483, 0x3746),
    (0x0483, 0x3747),
    (0x5050, 0x0100),
    (0x5050, 0x0101),
    (0x5050, 0x0102),
    (0x5050, 0x0103),
    (0x5050, 0x0104),
    (0x5050, 0x0105),
    (0x5050, 0x0106),
    (0x5050, 0x0107),
    (0x5050, 0x0300),
    (0x5050, 0x0301),
    (0x5050, 0x0400),
    (0x5050, 0x0500),
    (0x5050, 0x0700),
    (0x5050, 0x0800),
    (0x5050, 0x0900),
    (0x5050, 0x0A00),
    (0x5050, 0x0B00),
    (0x5050, 0x0C00),
    (0x5050, 0x0D00),
    (0x5050, 0x0E00),
    (0x5050, 0x0F00),
    (0x5050, 0x1000),
    (0x5050, 0x8000),
    (0x5050, 0x8001),
    (0x5050, 0x8002),
    (0x5050, 0x8003),
    (0x5050, 0x8004),
    (0x5050, 0x8005),
    (0x9E88, 0x9E8F),
    (0xDEEE, 0x0300),
    (0xDEEE, 0x02FF),
    (0xDEEE, 0x0302),
    (0xDEEE, 0x0303),
    (0x05D1, 0x1001),
    (0x05D1, 0x1002),
    (0x05D1, 0x1003),
    (0x05D1, 0x1004),
    (0x05D1, 0x1011),
    (0x05D1, 0x1013),
    (0x05D1, 0x2001),
    (0x05D1, 0x2002),
    (0x05D1, 0x2003),
    (0x05D1, 0x2011),
    (0x05D1, 0x2012),
    (0x05D1, 0x2021),
    (0x05D1, 0x2022),
    (0x05D1, 0x2023),
    (0x05D1, 0x2024),
    (0x05D1, 0x3011),
    (0x05D1, 0x3012),
    (0x05D1, 0x5001),
    (0x05D1, 0x6001),
    (0x05D1, 0x7001),
    (0x05D1, 0x8001),
    (0x05D1, 0x8002),
    (0x05D1, 0x8003),
    (0x05D1, 0x8004),
    (0x05D1, 0x9001),
    (0x05D1, 0x9002),
    (0x05D1, 0x9003),
    (0x05D1, 0x9004),
    (0x05D1, 0x9005),
    (0x05D1, 0x9006),
    (0x05D1, 0x9007),
    (0x05D1, 0x9008),
]
SILABS_VID_PID_GROUP = [
    (0x045B, 0x0053),
    (0x0471, 0x066A),
    (0x0489, 0xE000),
    (0x0489, 0xE003),
    (0x0745, 0x1000),
    (0x0846, 0x1100),
    (0x08E6, 0x5501),
    (0x08FD, 0x000A),
    (0x0BED, 0x1100),
    (0x0BED, 0x1101),
    (0x0FCF, 0x1003),
    (0x0FCF, 0x1004),
    (0x0FCF, 0x1006),
    (0x0FDE, 0xCA05),
    (0x10A6, 0xAA26),
    (0x10AB, 0x10C5),
    (0x10B5, 0xAC70),
    (0x2405, 0x0003),
    (0x10C5, 0xEA61),
    (0x10CE, 0xEA6A),
    (0x13AD, 0x9999),
    (0x1555, 0x0004),
    (0x166A, 0x0201),
    (0x166A, 0x0301),
    (0x166A, 0x0303),
    (0x166A, 0x0304),
    (0x166A, 0x0305),
    (0x166A, 0x0401),
    (0x166A, 0x0101),
    (0x16D6, 0x0001),
    (0x16DC, 0x0010),
    (0x16DC, 0x0011),
    (0x16DC, 0x0012),
    (0x16DC, 0x0015),
    (0x17A8, 0x0001),
    (0x17A8, 0x0005),
    (0x17F4, 0xAAAA),
    (0x1843, 0x0200),
    (0x18EF, 0xE00F),
    (0x1ADB, 0x0001),
    (0x1BE3, 0x07A6),
    (0x1E29, 0x0102),
    (0x1E29, 0x0501),
    (0x1FB9, 0x0100),
    (0x1FB9, 0x0200),
    (0x1FB9, 0x0201),
    (0x1FB9, 0x0202),
    (0x1FB9, 0x0203),
    (0x1FB9, 0x0300),
    (0x1FB9, 0x0301),
    (0x1FB9, 0x0302),
    (0x1FB9, 0x0303),
    (0x1FB9, 0x0400),
    (0x1FB9, 0x0401),
    (0x1FB9, 0x0402),
    (0x1FB9, 0x0403),
    (0x1FB9, 0x0404),
    (0x1FB9, 0x0600),
    (0x1FB9, 0x0601),
    (0x1FB9, 0x0602),
    (0x1FB9, 0x0700),
    (0x1FB9, 0x0701),
    (0x3195, 0xF190),
    (0x3195, 0xF280),
    (0x3195, 0xF281),
    (0x413C, 0x9500),
    (0x1908, 0x2311),
]
QINHENG_VID_PID_GROUP = [(0x4348, 0x5523)]
PROLIFIC_VID_PID_GROUP = [
    (0x04A5, 0x4027),
    (0x0557, 0x2008),
    (0x0547, 0x2008),
    (0x04BB, 0x0A03),
    (0x04BB, 0x0A0E),
    (0x056E, 0x5003),
    (0x056E, 0x5004),
    (0x0EBA, 0x1080),
    (0x0EBA, 0x2080),
    (0x0DF7, 0x0620),
    (0x0584, 0xB000),
    (0x2478, 0x2008),
    (0x1453, 0x4026),
    (0x0731, 0x0528),
    (0x6189, 0x2068),
    (0x11F7, 0x02DF),
    (0x04E8, 0x8001),
    (0x11F5, 0x0001),
    (0x11F5, 0x0003),
    (0x11F5, 0x0004),
    (0x11F5, 0x0005),
    (0x0745, 0x0001),
    (0x078B, 0x1234),
    (0x10B5, 0xAC70),
    (0x079B, 0x0027),
    (0x0413, 0x2101),
    (0x0E55, 0x110B),
    (0x0731, 0x2003),
    (0x050D, 0x0257),
    (0x058F, 0x9720),
    (0x11F6, 0x2001),
    (0x07AA, 0x002A),
    (0x05AD, 0x0FBA),
    (0x5372, 0x2303),
    (0x03F0, 0x0B39),
    (0x03F0, 0x3139),
    (0x03F0, 0x3239),
    (0x03F0, 0x3524),
    (0x04B8, 0x0521),
    (0x04B8, 0x0522),
    (0x054C, 0x0437),
    (0x11AD, 0x0001),
    (0x0B63, 0x6530),
    (0x0B8C, 0x2303),
    (0x110A, 0x1150),
    (0x0557, 0x2008),
]
