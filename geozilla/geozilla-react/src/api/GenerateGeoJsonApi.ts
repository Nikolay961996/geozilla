import {BaseApi} from "./BaseApi";
import {LatLngString} from "../types/LatLng";
import axios from "axios";

export class GenerateGeoJsonApi extends BaseApi {
    private static _generateGeoJsonUri = `${this.baseUri}/generate/geo-json`;


    public static async sendData(coords: LatLngString, file: File) {
        const formData = new FormData();
        formData.append("file", file);
        formData.append("latitude", coords.lat ?? "");
        formData.append("longitude", coords.lng ?? "");

        return await axios.post(
            this._generateGeoJsonUri,
            formData,
            {
                headers: {
                    "Content-Type": "multipart/form-data"
                },
                responseType: "blob"
            }
        );
    }
}