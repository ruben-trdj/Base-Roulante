import React from 'react';
import {StyleSheet, Text, View, TextInput, Button, Image, Switch, TouchableOpacity} from 'react-native'
import qs from 'qs';
import { Linking } from 'react-native';
import {createStackNavigator} from 'react-navigation-stack'

import Mouvement from "./Mouvement";
import {createAppContainer} from "react-navigation";

export default class App extends React.Component{
    render() {
        return(
            <Mouvement/>
        )
    }
}

