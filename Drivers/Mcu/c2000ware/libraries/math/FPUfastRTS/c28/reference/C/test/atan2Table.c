//#############################################################################
//! \file atan2Table.c
//! \brief  Arctangent Lookup Table (64) 
//! \author Vishal Coelho 
//! \date   03-Sep-2015
//! 
//
//  Group:			C2000
//  Target Family:	$DEVICE$
//
//#############################################################################
//
//
// 
// C2000Ware v5.03.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//This table has 195 entries
const float FastRTS_Arctan2Tbl[195] = {
	 0.0000000000000F, 	 1.0000406796754F, 	-0.0078110697504F,//   0 -> a0 a1 a2
	-0.0000038070221F, 	 1.0005280677716F, 	-0.0234103454930F,//   1 -> a0 a1 a2
	-0.0000189683103F, 	 1.0014985681064F, 	-0.0389411970749F,//   2 -> a0 a1 a2
	-0.0000528319270F, 	 1.0029436809648F, 	-0.0543585631597F,//   3 -> a0 a1 a2
	-0.0001124179960F, 	 1.0048507857875F, 	-0.0696182066495F,//   4 -> a0 a1 a2
	-0.0002042958852F, 	 1.0072032932429F, 	-0.0846770287235F,//   5 -> a0 a1 a2
	-0.0003344694932F, 	 1.0099808435012F, 	-0.0994933676284F,//   6 -> a0 a1 a2
	-0.0005082726057F, 	 1.0131595470233F, 	-0.1140272784299F,//   7 -> a0 a1 a2
	-0.0007302760352F, 	 1.0167122634486F, 	-0.1282407903432F,//   8 -> a0 a1 a2
	-0.0010042079796F, 	 1.0206089135568F, 	-0.1420981387153F,//   9 -> a0 a1 a2
	-0.0013328887174F, 	 1.0248168188011F, 	-0.1555659692585F,//  10 -> a0 a1 a2
	-0.0017181804313F, 	 1.0293010625688F, 	-0.1686135126670F,//  11 -> a0 a1 a2
	-0.0021609526118F, 	 1.0340248671352F, 	-0.1812127283288F,//  12 -> a0 a1 a2
	-0.0026610631593F, 	 1.0389499802219F, 	-0.1933384164099F,//  13 -> a0 a1 a2
	-0.0032173549768F, 	 1.0440370651602F, 	-0.2049682981460F,//  14 -> a0 a1 a2
	-0.0038276675461F, 	 1.0492460888683F, 	-0.2160830647060F,//  15 -> a0 a1 a2
	-0.0044888627021F, 	 1.0545367021923F, 	-0.2266663955068F,//  16 -> a0 a1 a2
	-0.0051968635789F, 	 1.0598686075727F, 	-0.2367049472787F,//  17 -> a0 a1 a2
	-0.0059467055004F, 	 1.0652019095265F, 	-0.2461883156129F,//  18 -> a0 a1 a2
	-0.0067325974217F, 	 1.0704974439939F, 	-0.2551089710223F,//  19 -> a0 a1 a2
	-0.0075479924132F, 	 1.0757170832221F, 	-0.2634621718398F,//  20 -> a0 a1 a2
	-0.0083856655986F, 	 1.0808240134986F, 	-0.2712458564756F,//  21 -> a0 a1 a2
	-0.0092377979239F, 	 1.0857829836932F, 	-0.2784605176930F,//  22 -> a0 a1 a2
	-0.0100960641391F, 	 1.0905605232115F, 	-0.2851090616505F,//  23 -> a0 a1 a2
	-0.0109517234066F, 	 1.0951251285570F, 	-0.2911966544475F,//  24 -> a0 a1 a2
	-0.0117957110286F, 	 1.0994474183038F, 	-0.2967305589200F,//  25 -> a0 a1 a2
	-0.0126187298731F, 	 1.1035002567840F, 	-0.3017199643105F,//  26 -> a0 a1 a2
	-0.0134113401986F, 	 1.1072588472740F, 	-0.3061758113249F,//  27 -> a0 a1 a2
	-0.0141640467111F, 	 1.1107007958875F, 	-0.3101106149475F,//  28 -> a0 a1 a2
	-0.0148673818311F, 	 1.1138061477170F, 	-0.3135382871760F,//  29 -> a0 a1 a2
	-0.0155119842982F, 	 1.1165573970591F, 	-0.3164739616554F,//  30 -> a0 a1 a2
	-0.0160886723953F, 	 1.1189394737663F, 	-0.3189338219481F,//  31 -> a0 a1 a2
	-0.0165885112292F, 	 1.1209397079557F, 	-0.3209349349913F,//  32 -> a0 a1 a2
	-0.0170028736449F, 	 1.1225477753665F, 	-0.3224950910217F,//  33 -> a0 a1 a2
	-0.0173234944993F, 	 1.1237556257493F, 	-0.3236326510678F,//  34 -> a0 a1 a2
	-0.0175425181404F, 	 1.1245573966328F, 	-0.3243664028596F,//  35 -> a0 a1 a2
	-0.0176525390650F, 	 1.1249493148104F, 	-0.3247154258333F,//  36 -> a0 a1 a2
	-0.0176466358231F, 	 1.1249295877727F, 	-0.3246989656827F,//  37 -> a0 a1 a2
	-0.0175183983443F, 	 1.1244982872665F, 	-0.3243363187867F,//  38 -> a0 a1 a2
	-0.0172619489202F, 	 1.1236572269554F, 	-0.3236467266133F,//  39 -> a0 a1 a2
	-0.0168719571636F, 	 1.1224098360981F, 	-0.3226492801386F,//  40 -> a0 a1 a2
	-0.0163436492943F, 	 1.1207610309185F, 	-0.3213628341191F,//  41 -> a0 a1 a2
	-0.0156728121663F, 	 1.1187170852579F, 	-0.3198059310243F,//  42 -> a0 a1 a2
	-0.0148557924513F, 	 1.1162855018516F, 	-0.3179967342794F,//  43 -> a0 a1 a2
	-0.0138894914410F, 	 1.1134748854835F, 	-0.3159529704599F,//  44 -> a0 a1 a2
	-0.0127713559110F, 	 1.1102948190320F, 	-0.3136918799664F,//  45 -> a0 a1 a2
	-0.0114993655128F, 	 1.1067557433290F, 	-0.3112301757137F,//  46 -> a0 a1 a2
	-0.0100720171306F, 	 1.1028688415306F, 	-0.3085840092872F,//  47 -> a0 a1 a2
	-0.0084883066548F, 	 1.0986459286453F, 	-0.3057689440639F,//  48 -> a0 a1 a2
	-0.0067477085786F, 	 1.0940993466476F, 	-0.3027999347241F,//  49 -> a0 a1 a2
	-0.0048501538149F, 	 1.0892418655233F, 	-0.2996913126099F,//  50 -> a0 a1 a2
	-0.0027960061186F, 	 1.0840865905167F, 	-0.2964567764093F,//  51 -> a0 a1 a2
	-0.0005860374406F, 	 1.0786468756663F, 	-0.2931093876086F,//  52 -> a0 a1 a2
	 0.0017785974517F, 	 1.0729362437272F, 	-0.2896615702293F,//  53 -> a0 a1 a2
	 0.0042963868040F, 	 1.0669683124404F, 	-0.2861251143513F,//  54 -> a0 a1 a2
	 0.0069654878940F, 	 1.0607567270642F, 	-0.2825111829598F,//  55 -> a0 a1 a2
	 0.0097837530165F, 	 1.0543150990086F, 	-0.2788303216685F,//  56 -> a0 a1 a2
	 0.0127487552067F, 	 1.0476569504397F, 	-0.2750924709426F,//  57 -> a0 a1 a2
	 0.0158578135459F, 	 1.0407956645818F, 	-0.2713069804113F,//  58 -> a0 a1 a2
	 0.0191080178933F, 	 1.0337444414765F, 	-0.2674826249277F,//  59 -> a0 a1 a2
	 0.0224962528869F, 	 1.0265162589889F, 	-0.2636276220892F,//  60 -> a0 a1 a2
	 0.0260192211618F, 	 1.0191238386511F, 	-0.2597496508597F,//  61 -> a0 a1 a2
	 0.0296734656363F, 	 1.0115796161864F, 	-0.2558558711062F,//  62 -> a0 a1 a2
	 0.0334553908381F, 	 1.0038957163225F, 	-0.2519529437632F,//  63 -> a0 a1 a2
	 0.0373612832121F, 	 0.9960839316110F, 	-0.2480470514256F,//  64 -> a0 a1 a2
}; 


// End of File