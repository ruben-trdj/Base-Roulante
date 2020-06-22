import React from 'react';
import {StyleSheet, Text, View, TextInput, Button, Image, Switch} from 'react-native';

const r = 0.04
const Vmax = Omegamax * r
const Omegamax = 10.5
const EpsilonMax = 12.5
const Amax =  EpsilonMax * r
const Esp = 0.3
const T0 = 10e-3



export default class App extends React.Component {
  constructor(props) {
    super(props)
    this.state = {
      instructions: [],
      currentInstruction: null,
        trajectoireLisse: false,
        ang: null,
        ray: null,
        a: null,
        instructionFile: []
    }
  }

  _genererTrajectoire(mat_vitesse) {
      alert(this.state.instructions)
  }

  _updateLIN(value) {
      const previousState = this.state.instructionFile
      const nextState = previousState.concat("LIN " + value)
      this.setState({instructionFile : nextState})
      alert(this.state.instructionFile)
  }
  _updateROT(value) {
      const previousState = this.state.instructionFile
      const nextState = previousState.concat("ROT " + value)
      this.setState({instructionFile : nextState})
      alert(this.state.instructionFile)
  }
  _updateCIRC(value) {
      const previousState = this.state.instructionFile
      const nextState = previousState.concat("LIN " + value)
      this.setState({instructionFile : nextState})
      alert(this.state.instructionFile)
  }

  _adaptateurArduino(mat_vitesse) {
      alert(JSON.stringify(mat_vitesse))
      let RNFS = require('react-native-fs');
      let path = RNFS.DocumentDirectoryPath + '/sortie_arduino.txt'

      for (let a = 0; a < 1; a ++) {
          for (let k=0 ; k< mat_vitesse.length; k++ ) {
              if (k === (mat_vitesse).length) {

              }
          }
      }
  }

  _decryptageMouvement() {

      let enter = [[0],[0]]
      let m = 0
      let t_init = 0
      let lissage = false

      for (let line in this.state.instructionFile){
          if (m==0 && line.includes("lissage")) {
              let index_debut = line.indexOf("(")
              let index_fin = line.indexOf(")")
              let parametre = line.slice(index_debut + 1, index_fin)

              if (parametre == "oui") {
                  lissage = true
              }
              m = m+1
          } else {
              let new_enter = [[0],[0]]
              let index_debut = line.indexOf("(")
              let index_fin = line.indexOf(")")
              let parametre = line.slice(index_debut + 1, index_fin)

              if (line.includes("LIN") || line.includes("lin")) {
                 new_enter = this._lin(parseFloat(parametre))
              }
              if (line.includes("ROT") || line.includes("rot")) {
                  new_enter = this._rot(parseFloat(parametre))
              }
              if (line.includes("CIRC") || line.includes("circ")) {
                  let separateur = line.indexOf(",")
                  if  (line.includes(",")){
                      let parametre_1 = parametre.slice(0,separateur)
                      let parametre_2 = parametre.slice(separateur + 1, parametre.length)
                      new_enter = this._circ(parseFloat(parametre_1),parseFloat(parametre_2))
                  }

              }
              m = m+1

              if (m>1) {
                  enter = enter.concat(enter,new_enter)
              }
              if (m>2 && lissage) {

              }
          }
          this._adaptateurArduino(enter)
      }


  }

  _ajoutInstruction() {
    const previousState = this.state.instructions
    const nextState = previousState.concat(this.state.currentInstruction)
    this.setState({instructions: nextState})
  }

  _ajoutInstructionCIRC() {
      if (this.state.ang !== null && this.state.ray !==null) {
          this.state.currentInstruction = "Cercle de " + this.state.ray + " mètre de diamètre à " + this.state.ang + " degrés \n"
          this._ajoutInstruction()
      } else {
          alert("rentrez des valeurs correctes")
      }
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
      const mat = [[]*nblin]
      for (let pas=0; pas<nblin; pas ++ ) {
          if (pas*T0 <= tau){
              let w = this.state.a *(Amax*pas*T0)/r
              mat[pas][0] = w
              mat[pas][1] = w
          } else if (pas*T0 > tau && pas*T0 < T){
              let w = this.state.a * (Vmax)/r
              mat[pas][0] = w
              mat[pas][1] = w
          } else {
              let w = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
              mat[pas][0] = w
              mat[pas][1] = w
          }
      }
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
      const mat = [[]*nblin]
      for (let pas=0; pas<nblin; pas ++ ) {
          if (pas*T0 < tau){
              let w = this.state.a *(Amax*pas*T0)/r
              mat[pas][0] = w
              mat[pas][1] = -w
          } else if (pas*T0 > tau && pas*T0 < T){
              let w = this.state.a * (Vmax)/r
              mat[pas][0] = w
              mat[pas][1] = -w
          } else {
              let w = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
              mat[pas][0] = w
              mat[pas][1] = -w
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
      const mat = [[]*nblin]
      const eta = (2*Math.abs(ray) - Esp)/(2*Math.abs(ray) + Esp)

      for (let pas=0; pas<nblin; pas ++ ) {
          if (pas*T0 < tau){
              let w1 = this.state.a *(Amax*pas*T0)/r
              let w2 = eta*w1
              if (ray > 0) {
                  mat[pas][0] = w1
                  mat[pas][1] = w2
              } else {
                  mat[pas][0] = w2
                  mat[pas][1] = w1
              }
          } else if (pas*T0 > tau && pas*T0 < T){
              let w = this.state.a * (Vmax)/r
              if (ray > 0) {
                  mat[pas][0] = w1
                  mat[pas][1] = w2
              } else {
                  mat[pas][0] = w2
                  mat[pas][1] = w1
              }
          } else {
              let w = this.state.a * (-Amax*pas*T0 + Amax*(T+tau))/r
              if (ray > 0) {
                  mat[pas][0] = w1
                  mat[pas][1] = w2
              } else {
                  mat[pas][0] = w2
                  mat[pas][1] = w1
              }
          }
      }
      alert(JSON.stringify(mat))
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
                  thumbColor={this.state.trajectoireLisse ? "#f5dd4b" : "#f4f3f4"}
                  ios_backgroundColor="#3e3e3e"
                  onValueChange={() => { if (this.state.trajectoireLisse === false) {
                              this.setState({trajectoireLisse: true})
                            } else {
                    this.setState({trajectoireLisse: false})
                  }
                  }}
                  value={this.state.trajectoireLisse}
              />

          </View>

          <View style={styles.row}>
            <TextInput placeholder="LIN" onChangeText={(value) => {
                 this.setState({currentInstruction: "Avancée de : " + value+ " mètres \n"});
                 this._updateLIN(value)
            }}   style={styles.input}/>
            <Button style={styles.addbutton} title="ajouter" onPress={() => this._ajoutInstruction()}/>
          </View>

          <View style={styles.row}>
            <TextInput placeholder="ROT" onChangeText={(text) => {
                this.setState({currentInstruction: "Rotation de : " + text+ " degrés \n"})
                this._updateROT(text)
            }}   style={styles.input}/>
            <Button style={styles.addbutton} title="ajouter" onPress={() => this._ajoutInstruction()}/>
          </View>

          <View style={styles.row}>
            <TextInput placeholder="ANG" onChangeText={(text) => this.setState({ang:text})}   style={styles.input1}/>
            <TextInput placeholder="RAY" onChangeText={(text) => this.setState({ray:text})}   style={styles.input2}/>

            <Button style={styles.addbutton2} title="ajouter" onPress={() => this._ajoutInstructionCIRC()}/>
          </View>

          <Text style={styles.instructions}>
            {this.state.instructions}
          </Text>


            <Button title="Générer la trajectoire" onPress={() => this._genererTrajectoire()}/>




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
