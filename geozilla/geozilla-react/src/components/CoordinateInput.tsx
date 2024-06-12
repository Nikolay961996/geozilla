import React from 'react';
import { TextField } from '@mui/material';
import {LatLngString} from "../types/LatLng";

interface CoordinateInputProps {
    selectedCoords: LatLngString | null;
    setSelectedCoords: React.Dispatch<React.SetStateAction<LatLngString | null>>;
}

const CoordinateInput : React.FC<CoordinateInputProps> = ({selectedCoords, setSelectedCoords}) => {

    return (
        <div>
            <TextField
                label="Широта"
                value={selectedCoords?.lat}
                onChange={(e) => setSelectedCoords({lat: e.target.value, lng: selectedCoords?.lng})}
            />
            <TextField
                label="Долгота"
                value={selectedCoords?.lng}
                onChange={(e) => setSelectedCoords({lat: selectedCoords?.lat, lng: e.target.value})}
            />
        </div>
    );
};

export default CoordinateInput;
