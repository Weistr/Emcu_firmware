//#############################################################################
//! \file expOutput.c
//! \brief  Exp Ouput Vector (512) 
//! \author Vishal Coelho 
//! \date   19-Jan-2017
//! 
//
//  Group:          C2000
//  Target Family:  $DEVICE$
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


const double test_golden[512] = {
     0.043213918264L,  0.043747500148L,  0.044287670410L,  0.044834510400L, 
     0.045388102472L,  0.045948529996L,  0.046515877374L,  0.047090230047L, 
     0.047671674514L,  0.048260298341L,  0.048856190172L,  0.049459439752L, 
     0.050070137927L,  0.050688376671L,  0.051314249089L,  0.051947849439L, 
     0.052589273140L,  0.053238616790L,  0.053895978182L,  0.054561456314L, 
     0.055235151407L,  0.055917164918L,  0.056607599561L,  0.057306559314L, 
     0.058014149441L,  0.058730476504L,  0.059455648383L,  0.060189774289L, 
     0.060932964781L,  0.061685331783L,  0.062446988603L,  0.063218049946L, 
     0.063998631933L,  0.064788852121L,  0.065588829517L,  0.066398684597L, 
     0.067218539326L,  0.068048517173L,  0.068888743134L,  0.069739343747L, 
     0.070600447112L,  0.071472182912L,  0.072354682430L,  0.073248078570L, 
     0.074152505878L,  0.075068100561L,  0.075995000508L,  0.076933345310L, 
     0.077883276281L,  0.078844936481L,  0.079818470737L,  0.080804025663L, 
     0.081801749683L,  0.082811793056L,  0.083834307892L,  0.084869448184L, 
     0.085917369823L,  0.086978230627L,  0.088052190362L,  0.089139410765L, 
     0.090240055572L,  0.091354290541L,  0.092482283476L,  0.093624204252L, 
     0.094780224842L,  0.095950519344L,  0.097135264004L,  0.098334637245L, 
     0.099548819692L,  0.100777994201L,  0.102022345886L,  0.103282062148L, 
     0.104557332698L,  0.105848349593L,  0.107155307260L,  0.108478402528L, 
     0.109817834654L,  0.111173805357L,  0.112546518846L,  0.113936181853L, 
     0.115343003660L,  0.116767196135L,  0.118208973762L,  0.119668553673L, 
     0.121146155679L,  0.122642002309L,  0.124156318838L,  0.125689333320L, 
     0.127241276629L,  0.128812382487L,  0.130402887502L,  0.132013031206L, 
     0.133643056086L,  0.135293207624L,  0.136963734332L,  0.138654887793L, 
     0.140366922694L,  0.142100096868L,  0.143854671331L,  0.145630910322L, 
     0.147429081343L,  0.149249455198L,  0.151092306037L,  0.152957911392L, 
     0.154846552225L,  0.156758512964L,  0.158694081550L,  0.160653549482L, 
     0.162637211855L,  0.164645367408L,  0.166678318570L,  0.168736371504L, 
     0.170819836153L,  0.172929026286L,  0.175064259549L,  0.177225857506L, 
     0.179414145696L,  0.181629453674L,  0.183872115067L,  0.186142467619L, 
     0.188440853245L,  0.190767618083L,  0.193123112544L,  0.195507691365L, 
     0.197921713664L,  0.200365542994L,  0.202839547394L,  0.205344099451L, 
     0.207879576351L,  0.210446359935L,  0.213044836763L,  0.215675398165L, 
     0.218338440304L,  0.221034364234L,  0.223763575962L,  0.226526486507L, 
     0.229323511964L,  0.232155073566L,  0.235021597745L,  0.237923516200L, 
     0.240861265962L,  0.243835289454L,  0.246846034566L,  0.249893954715L, 
     0.252979508920L,  0.256103161864L,  0.259265383971L,  0.262466651471L, 
     0.265707446475L,  0.268988257048L,  0.272309577279L,  0.275671907361L, 
     0.279075753660L,  0.282521628796L,  0.286010051717L,  0.289541547782L, 
     0.293116648833L,  0.296735893282L,  0.300399826186L,  0.304108999334L, 
     0.307863971328L,  0.311665307668L,  0.315513580835L,  0.319409370380L, 
     0.323353263008L,  0.327345852670L,  0.331387740651L,  0.335479535660L, 
     0.339621853920L,  0.343815319267L,  0.348060563235L,  0.352358225160L, 
     0.356708952269L,  0.361113399782L,  0.365572231011L,  0.370086117454L, 
     0.374655738905L,  0.379281783547L,  0.383964948065L,  0.388705937744L, 
     0.393505466575L,  0.398364257371L,  0.403283041864L,  0.408262560824L, 
     0.413303564167L,  0.418406811069L,  0.423573070079L,  0.428803119237L, 
     0.434097746188L,  0.439457748303L,  0.444883932801L,  0.450377116864L, 
     0.455938127766L,  0.461567802997L,  0.467266990386L,  0.473036548231L, 
     0.478877345428L,  0.484790261603L,  0.490776187240L,  0.496836023820L, 
     0.502970683957L,  0.509181091529L,  0.515468181826L,  0.521832901684L, 
     0.528276209629L,  0.534799076025L,  0.541402483217L,  0.548087425677L, 
     0.554854910160L,  0.561705955848L,  0.568641594511L,  0.575662870655L, 
     0.582770841687L,  0.589966578066L,  0.597251163472L,  0.604625694964L, 
     0.612091283147L,  0.619649052340L,  0.627300140742L,  0.635045700610L, 
     0.642886898425L,  0.650824915072L,  0.658860946018L,  0.666996201489L, 
     0.675231906656L,  0.683569301816L,  0.692009642583L,  0.700554200073L, 
     0.709204261096L,  0.717961128353L,  0.726826120629L,  0.735800572993L, 
     0.744885836998L,  0.754083280885L,  0.763394289792L,  0.772820265956L, 
     0.782362628930L,  0.792022815797L,  0.801802281379L,  0.811702498467L, 
     0.821724958034L,  0.831871169463L,  0.842142660774L,  0.852540978856L, 
     0.863067689696L,  0.873724378618L,  0.884512650520L,  0.895434130118L, 
     0.906490462186L,  0.917683311810L,  0.929014364634L,  0.940485327116L, 
     0.952097926784L,  0.963853912496L,  0.975755054706L,  0.987803145726L, 
     1.000000000000L,  1.012347454376L,  1.024847368381L,  1.037501624505L, 
     1.050312128478L,  1.063280809565L,  1.076409620850L,  1.089700539533L, 
     1.103155567229L,  1.116776730265L,  1.130566079990L,  1.144525693081L, 
     1.158657671859L,  1.172964144599L,  1.187447265859L,  1.202109216798L, 
     1.216952205508L,  1.231978467343L,  1.247190265260L,  1.262589890159L, 
     1.278179661223L,  1.293961926274L,  1.309939062123L,  1.326113474928L, 
     1.342487600557L,  1.359063904955L,  1.375844884516L,  1.392833066455L, 
     1.410031009197L,  1.427441302751L,  1.445066569112L,  1.462909462644L, 
     1.480972670490L,  1.499258912971L,  1.517770943996L,  1.536511551480L, 
     1.555483557760L,  1.574689820022L,  1.594133230731L,  1.613816718067L, 
     1.633743246364L,  1.653915816561L,  1.674337466648L,  1.695011272127L, 
     1.715940346476L,  1.737127841616L,  1.758576948386L,  1.780290897022L, 
     1.802272957649L,  1.824526440767L,  1.847054697752L,  1.869861121362L, 
     1.892949146247L,  1.916322249467L,  1.939983951012L,  1.963937814337L, 
     1.988187446896L,  2.012736500688L,  2.037588672801L,  2.062747705975L, 
     2.088217389164L,  2.114001558103L,  2.140104095893L,  2.166528933577L, 
     2.193280050738L,  2.220361476098L,  2.247777288122L,  2.275531615635L, 
     2.303628638440L,  2.332072587952L,  2.360867747833L,  2.390018454637L, 
     2.419529098463L,  2.449404123618L,  2.479648029282L,  2.510265370192L, 
     2.541260757322L,  2.572638858581L,  2.604404399513L,  2.636562164012L, 
     2.669116995042L,  2.702073795362L,  2.735437528270L,  2.769213218349L, 
     2.803405952220L,  2.838020879312L,  2.873063212637L,  2.908538229574L, 
     2.944451272665L,  2.980807750416L,  3.017613138118L,  3.054872978665L, 
     3.092592883393L,  3.130778532924L,  3.169435678021L,  3.208570140452L, 
     3.248187813874L,  3.288294664710L,  3.328896733057L,  3.370000133591L, 
     3.411611056487L,  3.453735768355L,  3.496380613182L,  3.539552013284L, 
     3.583256470279L,  3.627500566063L,  3.672290963801L,  3.717634408932L, 
     3.763537730183L,  3.810007840598L,  3.857051738582L,  3.904676508950L, 
     3.952889323997L,  4.001697444578L,  4.051108221201L,  4.101129095134L, 
     4.151767599526L,  4.203031360541L,  4.254928098506L,  4.307465629075L, 
     4.360651864406L,  4.414494814351L,  4.469002587664L,  4.524183393221L, 
     4.580045541257L,  4.636597444617L,  4.693847620024L,  4.751804689360L, 
     4.810477380965L,  4.869874530953L,  4.930005084541L,  4.990878097395L, 
     5.052502736999L,  5.114888284028L,  5.178044133753L,  5.241979797451L, 
     5.306704903840L,  5.372229200526L,  5.438562555477L,  5.505714958501L, 
     5.573696522758L,  5.642517486278L,  5.712188213505L,  5.782719196858L, 
     5.854121058310L,  5.926404550989L,  5.999580560796L,  6.073660108045L, 
     6.148654349124L,  6.224574578173L,  6.301432228786L,  6.379238875734L, 
     6.458006236706L,  6.537746174073L,  6.618470696679L,  6.700191961644L, 
     6.782922276201L,  6.866674099542L,  6.951460044700L,  7.037292880448L, 
     7.124185533219L,  7.212151089057L,  7.301202795581L,  7.391354063989L, 
     7.482618471070L,  7.575009761254L,  7.668541848679L,  7.763228819285L, 
     7.859084932941L,  7.956124625587L,  8.054362511411L,  8.153813385048L, 
     8.254492223809L,  8.356414189939L,  8.459594632896L,  8.564049091665L, 
     8.669793297097L,  8.776843174282L,  8.885214844941L,  8.994924629859L, 
     9.105989051341L,  9.218424835700L,  9.332248915777L,  9.447478433489L, 
     9.564130742415L,  9.682223410402L,  9.801774222219L,  9.922801182231L, 
    10.045322517110L, 10.169356678582L, 10.294922346203L, 10.422038430177L, 
    10.550724074198L, 10.680998658337L, 10.812881801960L, 10.946393366682L, 
    11.081553459358L, 11.218382435112L, 11.356900900401L, 11.497129716121L, 
    11.639090000745L, 11.782803133506L, 11.928290757618L, 12.075574783530L, 
    12.224677392233L, 12.375621038594L, 12.528428454742L, 12.683122653489L, 
    12.839726931797L, 12.998264874287L, 13.158760356789L, 13.321237549938L, 
    13.485720922817L, 13.652235246638L, 13.820805598475L, 13.991457365041L, 
    14.164216246508L, 14.339108260383L, 14.516159745419L, 14.695397365590L, 
    14.876848114097L, 15.060539317444L, 15.246498639543L, 15.434754085887L, 
    15.625334007767L, 15.818267106536L, 16.013582437940L, 16.211309416488L, 
    16.411477819882L, 16.614117793505L, 16.819259854956L, 17.026934898652L, 
    17.237174200475L, 17.450009422486L, 17.665472617689L, 17.883596234866L, 
    18.104413123453L, 18.327956538498L, 18.554260145661L, 18.783358026289L, 
    19.015284682545L, 19.250075042608L, 19.487764465930L, 19.728388748562L, 
    19.971984128545L, 20.218587291369L, 20.468235375495L, 20.720965977950L, 
    20.976817159988L, 21.235827452823L, 21.498035863432L, 21.763481880428L, 
    22.032205480008L, 22.304247131973L, 22.579647805825L, 22.858448976932L, 
}; 


// End of File