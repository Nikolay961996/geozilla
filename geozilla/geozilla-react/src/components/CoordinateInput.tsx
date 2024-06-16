import React from 'react';
import {Box, Grid, TextField} from '@mui/material';
import {LatLngString} from "../types/LatLng";

interface CoordinateInputProps {
    selectedCoords: LatLngString | null;
    setSelectedCoords: React.Dispatch<React.SetStateAction<LatLngString | null>>;
}

const CoordinateInput : React.FC<CoordinateInputProps> = ({selectedCoords, setSelectedCoords}) => {

    return (
        <div>
            <Grid container style={{justifyContent: 'center'}}>
                <Grid item xs={6}>
                    <TextField
                        label="Широта"
                        value={selectedCoords?.lat}
                        onChange={(e) => setSelectedCoords({lat: e.target.value, lng: selectedCoords?.lng})}
                    />
                </Grid>
                <Grid item xs={6}>
                    <TextField
                        label="Долгота"
                        value={selectedCoords?.lng}
                        onChange={(e) => setSelectedCoords({lat: selectedCoords?.lat, lng: e.target.value})}
                    />
                </Grid>
            </Grid>
            <Box margin={1}>

            </Box>
            <Box margin={1}>

            </Box>
        </div>
    );
};

export default CoordinateInput;
