//#############################################################################
//! \file   golden.c
//! \brief  Powf Ouput Vector (512) 
//! \author Vishal Coelho 
//! \date   07-Feb-2017
//! 
//
//  Group:          C2000
//  Target Family:  $DEVICE$
//
//#############################################################################
//
//
// $Copyright: Copyright (C) 2024 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#include "fastrts.h"

const float32_t test_golden[512] = {
     1.00000000000F,  0.98702291954F,  0.97436674897F,  0.96202239279F, 
     0.94998106830F,  0.93823429344F,  0.92677387514F,  0.91559189829F, 
     0.90468071504F,  0.89403293466F,  0.88364141380F,  0.87349924716F, 
     0.86359975853F,  0.85393649232F,  0.84450320525F,  0.83529385860F, 
     0.82630261064F,  0.81752380939F,  0.80895198576F,  0.80058184688F, 
     0.79240826975F,  0.78442629514F,  0.77663112174F,  0.76901810051F, 
     0.76158272935F,  0.75432064787F,  0.74722763243F,  0.74029959138F, 
     0.73353256050F,  0.72692269852F,  0.72046628300F,  0.71415970615F, 
     0.70799947101F,  0.70198218767F,  0.69610456963F,  0.69036343038F, 
     0.68475568002F,  0.67927832207F,  0.67392845038F,  0.66870324616F, 
     0.66359997510F,  0.65861598466F,  0.65374870137F,  0.64899562833F, 
     0.64435434273F,  0.63982249346F,  0.63539779891F,  0.63107804469F, 
     0.62686108158F,  0.62274482345F,  0.61872724534F,  0.61480638155F, 
     0.61098032382F,  0.60724721955F,  0.60360527017F,  0.60005272944F, 
     0.59658790191F,  0.59320914142F,  0.58991484958F,  0.58670347442F, 
     0.58357350898F,  0.58052349004F,  0.57755199681F,  0.57465764972F, 
     0.57183910926F,  0.56909507481F,  0.56642428356F,  0.56382550944F, 
     0.56129756209F,  0.55883928588F,  0.55644955896F,  0.55412729231F, 
     0.55187142886F,  0.54968094267F,  0.54755483802F,  0.54549214866F, 
     0.54349193705F,  0.54155329356F,  0.53967533577F,  0.53785720776F, 
     0.53609807948F,  0.53439714603F,  0.53275362707F,  0.53116676621F, 
     0.52963583038F,  0.52816010934F,  0.52673891504F,  0.52537158116F, 
     0.52405746255F,  0.52279593477F,  0.52158639358F,  0.52042825448F, 
     0.51932095231F,  0.51826394074F,  0.51725669191F,  0.51629869601F, 
     0.51538946090F,  0.51452851169F,  0.51371539046F,  0.51294965581F, 
     0.51223088261F,  0.51155866161F,  0.51093259917F,  0.51035231691F, 
     0.50981745147F,  0.50932765416F,  0.50888259076F,  0.50848194119F, 
     0.50812539931F,  0.50781267265F,  0.50754348215F,  0.50731756198F, 
     0.50713465929F,  0.50699453402F,  0.50689695867F,  0.50684171814F, 
     0.50682860951F,  0.50685744187F,  0.50692803617F,  0.50704022501F, 
     0.50719385253F,  0.50738877423F,  0.50762485681F,  0.50790197807F, 
     0.50822002674F,  0.50857890239F,  0.50897851526F,  0.50941878619F, 
     0.50989964649F,  0.51042103781F,  0.51098291210F,  0.51158523147F, 
     0.51222796810F,  0.51291110420F,  0.51363463186F,  0.51439855306F, 
     0.51520287951F,  0.51604763268F,  0.51693284365F,  0.51785855313F, 
     0.51882481135F,  0.51983167804F,  0.52087922239F,  0.52196752302F, 
     0.52309666790F,  0.52426675438F,  0.52547788911F,  0.52673018807F, 
     0.52802377649F,  0.52935878890F,  0.53073536907F,  0.53215367003F, 
     0.53361385404F,  0.53511609263F,  0.53666056657F,  0.53824746589F, 
     0.53987698988F,  0.54154934714F,  0.54326475556F,  0.54502344234F, 
     0.54682564405F,  0.54867160663F,  0.55056158544F,  0.55249584528F, 
     0.55447466042F,  0.55649831469F,  0.55856710147F,  0.56068132377F, 
     0.56284129427F,  0.56504733540F,  0.56729977935F,  0.56959896820F, 
     0.57194525393F,  0.57433899850F,  0.57678057396F,  0.57927036246F, 
     0.58180875642F,  0.58439615851F,  0.58703298183F,  0.58971964993F, 
     0.59245659695F,  0.59524426769F,  0.59808311773F,  0.60097361350F, 
     0.60391623245F,  0.60691146309F,  0.60995980516F,  0.61306176971F, 
     0.61621787925F,  0.61942866785F,  0.62269468132F,  0.62601647726F, 
     0.62939462527F,  0.63282970707F,  0.63632231664F,  0.63987306035F, 
     0.64348255716F,  0.64715143872F,  0.65088034959F,  0.65466994735F, 
     0.65852090280F,  0.66243390015F,  0.66640963714F,  0.67044882528F, 
     0.67455219000F,  0.67872047087F,  0.68295442176F,  0.68725481107F, 
     0.69162242192F,  0.69605805237F,  0.70056251561F,  0.70513664021F, 
     0.70978127031F,  0.71449726589F,  0.71928550296F,  0.72414687383F, 
     0.72908228734F,  0.73409266912F,  0.73917896181F,  0.74434212539F, 
     0.74958313735F,  0.75490299306F,  0.76030270598F,  0.76578330797F, 
     0.77134584955F,  0.77699140025F,  0.78272104886F,  0.78853590375F, 
     0.79443709321F,  0.80042576572F,  0.80650309034F,  0.81267025699F, 
     0.81892847679F,  0.82527898246F,  0.83172302863F,  0.83826189221F, 
     0.84489687274F,  0.85162929283F,  0.85846049846F,  0.86539185944F, 
     0.87242476976F,  0.87956064804F,  0.88680093789F,  0.89414710842F, 
     0.90160065458F,  0.90916309765F,  0.91683598568F,  0.92462089398F, 
     0.93251942551F,  0.94053321144F,  0.94866391160F,  0.95691321496F, 
     0.96528284017F,  0.97377453606F,  0.98239008218F,  0.99113128930F, 
     1.00000000000F,  1.00899808921F,  1.01812746479F,  1.02739006808F, 

}; 

//End of File