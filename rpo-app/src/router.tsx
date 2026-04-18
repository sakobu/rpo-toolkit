import { createBrowserRouter } from 'react-router';

import { MissionStub } from './views/MissionStub';
import { SetupView } from './views/SetupView';
import { Shell } from './views/Shell';
import { TransferStub } from './views/TransferStub';

export const router = createBrowserRouter([
  {
    path: '/',
    Component: Shell,
    children: [
      { index: true, Component: SetupView },
      { path: 'transfer', Component: TransferStub },
      { path: 'mission', Component: MissionStub },
    ],
  },
]);
