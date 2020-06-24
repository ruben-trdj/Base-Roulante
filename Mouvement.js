import React from 'react';
import {StyleSheet, Text, View, TextInput, Button, Image, Switch, TouchableOpacity} from 'react-native'
import qs from 'qs';
import { Linking } from 'react-native';


const r = 0.04
const Omegamax = 10.5
const Vmax = Omegamax * r
const EpsilonMax = 12.5
const Amax =  EpsilonMax * r
const Esp = 0.3
const T0 = 10e-3



export default class Mouvement extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
            instructions: [],
            currentInstruction: null,
            lissage: false,
            ang: null,
            ray: null,
            a: null,
            instructionFile: [],
            lin: null,
            rot: null,
            abs: null,
            ord: null,
        }
    }

    async sendEmail(to, subject, body, options = {}) {
        const {cc, bcc} = options;
        let url = `mailto:${to}`;

        // Create email link query
        const query = qs.stringify({
            subject: subject,
            body: body,
            cc: cc,
            bcc: bcc
        });

        if (query.length) {
            url += `?${query}`;
        }

        // check if we can use this link
        const canOpen = await Linking.canOpenURL(url);

        if (!canOpen) {
            throw new Error('Provided URL can not be handled');
        }

        return Linking.openURL(url);
    }


    _updateLIN() {
        const previousState = this.state.instructionFile
        const nextState = previousState.concat(this.state.lin)
        this.setState({instructionFile : nextState})
    }
    _updateROT() {
        const previousState = this.state.instructionFile
        const nextState = previousState.concat(this.state.rot)
        this.setState({instructionFile : nextState})
    }
    _updateCIRC() {
        const previousState = this.state.instructionFile
        const nextState = previousState.concat("CIRC("+this.state.ang+","+this.state.ray+")")
        this.setState({instructionFile : nextState})
    }

    _updatePOINT() {
        const previousState = this.state.instructionFile
        const nextState = previousState.concat("(" + this.state.abs +","+ this.state.ord +")")
        this.setState({instructionFile : nextState})
    }

    _adaptateurArduino(mat_vitesse) {

        for (let pas = 0; pas < mat_vitesse.length; pas = pas + 1 ) {

            if (mat_vitesse[pas] == 0){
                mat_vitesse[pas] = 127
            } else if (mat_vitesse[pas] > 0 ) {
                mat_vitesse[pas] = Math.trunc(mat_vitesse[pas]*255/21) + 128
            } else {
                mat_vitesse[pas] = Math.trunc(mat_vitesse[pas]*255/21) + 127
            }
        }
        this.sendEmail("bouteteliot@gmail.com",'Test',JSON.stringify(mat_vitesse))
            .then(() => alert("email envoyé "))
    }

    _decryptageMouvement() {

        let enter = [0,0];
        let t_init = 0;

        for (let i = 0, len = this.state.instructionFile.length; i < len; i++) {
            let instruction = this.state.instructionFile[i]

            let new_enter = [[0],[0]]
            let index_debut = instruction.indexOf("(")
            let index_fin = instruction.indexOf(")")
            let parametre = instruction.slice(index_debut + 1, index_fin)

            if (instruction.includes("LIN") || instruction.includes("lin")) {
                new_enter = this._lin(parseFloat(parametre))

            }
            if (instruction.includes("ROT") || instruction.includes("rot")) {
                new_enter = this._rot(parseFloat(parametre))
            }
            if (instruction.includes("CIRC") || instruction.includes("circ")) {
                let separateur = instruction.indexOf(",")

                if  (instruction.includes(",")){
                    let parametre_1 = instruction.slice(5,separateur)
                    let parametre_2 = instruction.slice(separateur + 1, instruction.length - 1)
                    new_enter = this._circ(parseFloat(parametre_1),parseFloat(parametre_2))
                }

            }
            if (this.state.instructionFile.length>=1) {
                enter = enter.concat(enter,new_enter)

            }
            if (this.state.instructionFile.length>2 && this.state.lissage) {
                let values = this._lissageTrajectoire(enter, t_init)
                enter = values[0]
                t_init = values[1]


            }

        }

        this._adaptateurArduino(enter)

    }

    _lissageTrajectoire(mat_vitesse, t_init) {
        let mat_vitesse_pos = Math.abs(mat_vitesse)
        let t1 = t_init
        while (mat_vitesse_pos[2*(t1+1)] >= mat_vitesse_pos[2*t1]) {
            t1 = t1 + 1
        }
        let t2 = t1
        while (mat_vitesse_pos[2*(t2+1)] <= mat_vitesse_pos[2*t2]) {
            t2 = t2 + 1
        }
        let t3 = t2 + 1
        while (mat_vitesse_pos[2*(t3+1)] > mat_vitesse_pos[2*t3]) {
            t3 = t3 + 1
        }
        let delta_t1 = t2 - t1
        let delta_t2 = t3 - t2

        if (delta_t1 > delta_t2) {
            let nblin = (mat_vitesse.length/2) - delta_t2
            let new_mat_vitesse = []
            for (let pas=0; pas<nblin; pas ++){
                new_mat_vitesse.push(0,0)
            }
            new_mat_vitesse.fill(mat_vitesse.slice(0,(t2 - delta_t2)*2), 0,(t2 - delta_t2)*2 )
            new_mat_vitesse.fill(mat_vitesse.slice(t3*2,mat_vitesse.length), t2*2,new_mat_vitesse.length)

            for (let i=0; i < delta_t2; i = i +2 ) {
                new_mat_vitesse[2*(t2 - delta_t2) + i] = mat_vitesse[2*(t2 - delta_t2) + i] + mat_vitesse[2*t2 + i]
            }
            return [new_mat_vitesse, t2]
        } else {
            let nblin = (mat_vitesse.length/2) - delta_t1
            let new_mat_vitesse = []
            for (let pas=0; pas<nblin; pas ++){
                new_mat_vitesse.push(0,0)
            }
            new_mat_vitesse.fill(mat_vitesse.slice(0,t1*2), 0, t1*2 )
            new_mat_vitesse.fill(mat_vitesse.slice(2*(t2+delta_t1)), 2*t2,new_mat_vitesse.length)


            for (let i=0; i < delta_t1; i = i +2 ) {
                new_mat_vitesse[2*(t1) + i] = mat_vitesse[2*t1 + i] + mat_vitesse[2*t2 + i]
            }
            return [new_mat_vitesse, t2]
        }




    }



    _ajoutInstruction() {
        const previousState = this.state.instructions
        const nextState = previousState.concat(this.state.currentInstruction)
        this.setState({instructions: nextState})
    }

    _ajoutInstructionCIRC() {
        if (this.state.ang !== null && this.state.ray !==null) {
            this.state.currentInstruction = "Cercle de " + this.state.ang + " degré de rotation avec " + this.state.ray + " mètre de rayon\n"
            this._ajoutInstruction()
        } else {
            alert("rentrez des valeurs correctes")
        }
    }

    _ajoutInstructionPOINTS() {
        if (this.state.abs !== null && this.state.ord !== null){
            this.state.currentInstruction = "(" + this.state.abs + "," + this.state.ord +")\n"
            this._ajoutInstruction()
        } else {
            alert("rentrez des valeurs correctes")
        }
    }

    _decryptagePoints() {
        let enter = [0,0];
        let mat_co = [0,0,0,0]
        let alpha = Math.pow(10, 1000)
        let angle = 90
        let t_init = 0
        let phi
        let longueur

        for (let i = 0, len = this.state.instructionFile.length; i < len; i++) {
            let instruction = this.state.instructionFile[i]



            if (this.state.instructionFile.length >= 1 ) {
                let separateur = instruction.indexOf(",")
                let new_enter_rot = [0,0]
                let new_enter_lin = [0,0]

                mat_co[0] =  mat_co[2]
                mat_co[1] = mat_co[3]
                mat_co[2] = parseFloat(instruction.substring(1,separateur))
                mat_co[3] = parseFloat(instruction.substring(separateur+1,instruction.length))

                let values = this._fromPointsToMvt(mat_co, alpha)
                phi = values[0]
                longueur = values[1]
                alpha = values[2]


                new_enter_rot = this._rot(angle - phi*180/Math.PI)
                new_enter_lin = this._lin(longueur)

                angle = phi*180/Math.PI
                enter = enter.concat(new_enter_rot)
                enter = enter.concat(new_enter_lin)
            }

            if (this.state.instructionFile.length >= 2 && this.state.lissage){
                let values2 = this._lissageTrajectoire(enter, t_init)
                enter = values2[0]
                t_init = values2[1]

            }

        }
        this._adaptateurArduino(enter)
    }

    _fromPointsToMvt(mat_co, alpha) {
        let phi
        if (mat_co[1] - mat_co[0] == 0) {
            if (mat_co[3] > mat_co[1]) {
                phi = Math.PI/2
            } else {
                phi = - Math.PI/2
            }
        } else {
            phi = Math.atan((mat_co[3] - mat_co[1]) / (mat_co[2] - mat_co[0]))
        }
        let longueur = Math.sqrt((mat_co[3] - mat_co[1])**2 + (mat_co[2] - mat_co[0])**2)

        if ((mat_co[3] > alpha * (mat_co[2] - mat_co[0]) + mat_co[1]) && ((mat_co[2] - mat_co[0]) != 0)) {
            phi = phi + Math.PI
        }
        if (mat_co[2] - mat_co[0] == 0) {
            alpha = Math.pow(10, 1000)
        } else {
            alpha = (mat_co[3] - mat_co[1]) / (mat_co[2] - mat_co[0])
        }

        return [phi,longueur,alpha]

    }

    _lin(dist){


        if (dist>0) {
            this.state.a = 1
        } else {
            this.state.a = -1
        }
        let T = Math.abs(dist/Vmax)

        let tau = Vmax/Amax

        if (T<tau) {
            tau = Math.sqrt(Math.abs(dist/Amax))
            T = tau
        }
        const nblin = Math.trunc((T+tau)/T0)

        const mat = []
        for (let pas=0; pas<nblin/2; pas ++){
            mat.push(0,0)
        }



        for (let pas=0; pas<nblin; pas = pas + 2 ) {
            if (pas*T0 <= tau){
                let w = this.state.a *(Amax*pas*T0)/r
                mat[pas] = w
                mat[pas+1] = w
            } else if (pas*T0 > tau && pas*T0 < T){
                let w = this.state.a * (Vmax)/r
                mat[pas] = w
                mat[pas+1] = w
            } else {
                let w = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
                mat[pas] = w
                mat[pas+1] = w
            }
        }

        return(mat)
    }

    _rot(ang) {
        if (ang>0) {
            this.state.a = 1
        } else {
            this.state.a = -1
        }
        let T = (Math.abs(ang)*(Math.PI/360)*Esp)/Vmax
        let tau = Vmax/Amax
        if (T<tau) {
            tau = Math.sqrt((Math.abs(ang)*(Math.PI/360)*Esp)/Amax)
            T = tau
        }
        const nblin = Math.trunc((T+tau)/T0)
        const mat = []
        for (let pas=0; pas<nblin/2; pas ++){
            mat.push(0,0)
        }

        for (let pas=0; pas<nblin; pas = pas + 2 ) {
            if (pas*T0 < tau){
                let w = this.state.a *(Amax*pas*T0)/r
                mat[pas] = w
                mat[pas+1] = -w
            } else if (pas*T0 > tau && pas*T0 < T){
                let w = this.state.a * (Vmax)/r
                mat[pas] = w
                mat[pas+1] = -w
            } else {
                let w = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
                mat[pas] = w
                mat[pas+1] = -w
            }
        }

        return mat
    }

    _circ(ang, ray) {

        this.state.a = ang > 0 ? 1 : -1;

        let T = (Math.abs(ang)*(Math.PI/360)*(2*Math.abs(ray) + Esp))/Vmax;
        let tau = Vmax/Amax
        if (T<tau) {
            tau = Math.sqrt((Math.abs(ang)*(Math.PI/360)*(2*Math.abs(ray) + Esp))/Amax)
            T = tau
        }
        const nblin = Math.trunc((T+tau)/T0)
        const mat = []
        for (let pas=0; pas<nblin/2; pas ++){
            mat.push(0,0)
        }
        const eta = (2*Math.abs(ray) - Esp)/(2*Math.abs(ray) + Esp)


        for (let pas=0; pas<nblin; pas = pas + 2 ) {
            if (pas*T0 < tau){
                let w1 = this.state.a *(Amax*pas*T0)/r
                let w2 = eta*w1
                if (ray > 0) {
                    mat[pas] = w1
                    mat[pas+1] = w2
                } else {
                    mat[pas] = w2
                    mat[pas+1] = w1
                }

            } else if (pas*T0 > tau && pas*T0 < T){
                let w1 = this.state.a * (Vmax)/r
                let w2 = eta*w1
                if (ray > 0) {
                    mat[pas] = w1
                    mat[pas+1] = w2
                } else {
                    mat[pas] = w2
                    mat[pas+1] = w1
                }

            } else {
                let w1 = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
                let w2 = eta*w1
                if (ray > 0) {
                    mat[pas] = w1
                    mat[pas+1] = w2
                } else {
                    mat[pas] = w2
                    mat[pas+1] = w1
                }

            }

        }
        return mat
    }



    render() {
        return(
            <View style={ styles.container}>
                <Text style={styles.title}> Entrez ci dessous vos instructions </Text>

                <Image source={require('../BaseRoulante/assets/BaseRoulanteImage.jpg')} style={styles.image}/>

                <View style={{flexDirection: "row"}}>

                    <Text>
                        Trajectoire lisse :
                    </Text>
                    <Switch
                        style={styles.switch}
                        trackColor={{ false: "#767577", true: "#81b0ff" }}
                        thumbColor={this.state.lissage ? "#f5dd4b" : "#f4f3f4"}
                        ios_backgroundColor="#3e3e3e"
                        onValueChange={() => { if (this.state.lissage === false) {
                            this.setState({lissage: true})
                        } else {
                            this.setState({lissage: false})
                        }
                        }}
                        value={this.state.lissage}
                    />

                </View>

                <View style={styles.row}>
                    <TextInput placeholder="LIN" onChangeText={(value) => {
                        this.setState({currentInstruction: "Avancée de : " + value+ " mètres \n", lin : "LIN(" + value + ")"});
                    }}   style={styles.input}/>
                    <Button style={styles.addbutton} title="ajouter" onPress={() => {
                        this._ajoutInstruction()
                        this._updateLIN()

                    }}/>
                </View>

                <View style={styles.row}>
                    <TextInput placeholder="ROT" onChangeText={(text) => {
                        this.setState({currentInstruction: "Rotation de : " + text+ " degrés \n", rot: "ROT(" + text + ")"})
                    }}   style={styles.input}/>
                    <Button style={styles.addbutton} title="ajouter" onPress={() => {
                        this._ajoutInstruction()
                        this._updateROT()
                    }}/>
                </View>

                <View style={styles.row}>
                    <TextInput placeholder="ANG" onChangeText={(text) => this.setState({ang:text})}   style={styles.input1}/>
                    <TextInput placeholder="RAY" onChangeText={(text) => this.setState({ray:text})}   style={styles.input2}/>

                    <Button style={styles.addbutton2} title="ajouter" onPress={() => {
                        this._ajoutInstructionCIRC()
                        this._updateCIRC()
                    }}/>
                </View>

                <View style={styles.row}>
                    <TextInput placeholder="Abs" onChangeText={(text) => this.setState({abs:text})}   style={styles.input1}/>
                    <TextInput placeholder="Ord" onChangeText={(text) => this.setState({ord:text})}   style={styles.input2}/>

                    <Button style={styles.addbutton2} title="ajouter" onPress={() => {
                        this._ajoutInstructionPOINTS()
                        this._updatePOINT()
                    }}/>
                </View>

                <Text style={styles.instructions}>
                    {this.state.instructions}
                </Text>


                <Button title="Générer la trajectoire avec les instructions" onPress={() => this._decryptageMouvement()}/>
                <Button title="Générer la trajectoire avec les points" onPress={() => this._decryptagePoints()}/>


            </View>


        )
    }
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
    },
    switch: {
        marginBottom: 40,
        marginLeft: 20,
        marginTop:-4
    },
    row: {
        flexDirection: "row"
    },
    input: {
        borderWidth: 1,
        borderColor: "black",
        marginRight: 90,
        height : 30,
        width: 100,
        marginLeft: 50
    },
    input2: {
        borderWidth: 1,
        borderColor: "black",
        marginRight: 87,
        width : 40,
        marginLeft: -20
    },
    title:{
        marginBottom: 100
    },
    addbutton: {
        marginLeft : 100,
        margin : 100
    },
    image:{
        marginRight: 50,
        marginTop: -80
    },
    instructions: {
        marginTop: 20
    },
    addbutton2: {
        marginLeft : 120,
        margin : 100
    },
    input1: {
        borderWidth: 1,
        borderColor: "black",
        marginRight: 45,
        marginLeft : 49,
        width: 40
    }
});
